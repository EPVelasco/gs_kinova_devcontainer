FROM osrf/ros:noetic-desktop-focal

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user (with passwordless sudo)
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,utility,compute
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp
ENV DEBIAN_FRONTEND=noninteractive

# Base updates and essentials
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      curl wget ca-certificates gnupg2 bash-completion build-essential apt-utils \
      python3 python3-pip python3-tk python3-distutils python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS meta package (pinned) and common ROS tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-desktop-full=1.5.0-1* \
      ros-noetic-perception-pcl \
      ros-noetic-rviz \
      ros-noetic-eigen-conversions \
      ros-noetic-ros-controllers \
      ros-noetic-moveit* \
      ros-noetic-ompl \
      ros-noetic-gazebo-ros-pkgs \
      ros-noetic-joy \
      python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
      iputils-ping screen vim nano \
    && rm -rf /var/lib/apt/lists/*

# Extra graphics / OpenGL bits
RUN apt-get update && apt-get install -y --no-install-recommends \
      libosmesa6-dev libgl1-mesa-dev libgl1-mesa-glx libglfw3 libglfw3-dev libglew-dev \
      libglu1-mesa mesa-utils \
      # Qt/X11 helpers often needed by gzclient
      libxkbcommon-x11-0 libxcb-xinerama0 libqt5svg5 libqt5x11extras5 \
    && rm -rf /var/lib/apt/lists/*

# Reactivate ROS apt key (focal)
RUN sed -i '/packages.ros.org/s/^/#/' /etc/apt/sources.list /etc/apt/sources.list.d/*.list || true && \
    apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && \
    rm -rf /var/lib/apt/lists/* && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros1-latest-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# Python runtimes & scientific stack you requested
# NOTE: Use python3 -m pip and pin versions for reproducibility
RUN python3 -m pip install --no-cache-dir \
      numpy==1.23.5 \
      scipy==1.10.1 \
      opencv-python==4.8.1.78 \
      opencv-contrib-python==4.8.1.78 \
      pygame==2.5.2

# Dev helpers (ripgrep, fd, unzip, xclip) installed as root
RUN apt-get update && apt-get install -y --no-install-recommends \
      ripgrep fd-find unzip xclip \
    && rm -rf /var/lib/apt/lists/*

# Node.js 23 (for LSP tools like pyright via Mason if you add later)
RUN curl -fsSL https://deb.nodesource.com/setup_23.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    rm -rf /var/lib/apt/lists/*

# Neovim build deps + build from source (no glibc issues)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ninja-build gettext cmake unzip curl build-essential git \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/neovim/neovim.git /tmp/neovim && \
    cd /tmp/neovim && make CMAKE_BUILD_TYPE=Release && make install && \
    rm -rf /tmp/neovim

# Conan 1.59 (root)
RUN python3 -m pip install --no-cache-dir conan==1.59 && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-effort-controllers* ros-noetic-serial \
    && rm -rf /var/lib/apt/lists/*

# Switch to user
ENV HOME=/home/${USERNAME}
USER $USERNAME
WORKDIR $HOME

# Conan user config
RUN conan config set general.revisions_enabled=1 && \
    conan profile new default --detect > /dev/null && \
    conan profile update settings.compiler.libcxx=libstdc++11 default

# Neovim config from your repo
RUN git clone --depth 1 https://github.com/lfrecalde1/nvim_setup /tmp/nvim_setup && \
    mkdir -p "$HOME/.config/nvim" && \
    rm -rf "$HOME/.config/nvim/lua" && \
    cp -r /tmp/nvim_setup/lua "$HOME/.config/nvim/" && \
    cp /tmp/nvim_setup/init.lua "$HOME/.config/nvim/" && \
    rm -rf /tmp/nvim_setup && \
    ls -la "$HOME/.config/nvim"

# Ensure local bin exists and fd symlink is available for your user
RUN mkdir -p "$HOME/.local/bin" && \
    ln -sf "$(command -v fdfind)" "$HOME/.local/bin/fd" || true

#git checkout 37e17d31890ab54e5a855f1fe787fbf2f5d43bdb && \
# --- Build Acados ---
# NOTE: Pin acados to the exact commit (uncommented from your comment above)
RUN git clone https://github.com/acados/acados.git && \
    cd acados && \
    git checkout 37e17d31890ab54e5a855f1fe787fbf2f5d43bdb && \
    git submodule update --recursive --init && \
    mkdir -p build && cd build && \
    cmake -DACADOS_WITH_DAQP=ON -DACADOS_INSTALL_DIR="${HOME}/acados" .. && \
    make install -j"$(nproc)" && \
    python3 -m pip install --no-cache-dir -e "${HOME}/acados/interfaces/acados_template"

# --- Python tools ---
# NOTE: Pin CasADi to 3.7.1 (matches the machine that works)
RUN python3 -m pip install --no-cache-dir pyyaml==6.0.1 pynput==1.7.6 osqp==0.6.5 casadi==3.7.1

# --- Build tera_renderer ---
RUN git clone https://github.com/acados/tera_renderer && \
    cd tera_renderer && \
    curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    /bin/bash -lc "source ${HOME}/.cargo/env && cargo build --verbose --release" && \
    cp ${HOME}/tera_renderer/target/release/t_renderer ${HOME}/acados/bin/t_renderer

# Shell cosmetics and ROS sourcing for the user
RUN echo "TERM=xterm-256color" >> ~/.bashrc && \
    echo "# COLOR Text" >> ~/.bashrc && \
    echo "PS1='\\[\\033[01;33m\\]\\u\\[\\033[01;33m\\]@\\[\\033[01;33m\\]\\h\\[\\033[01;34m\\]:\\[\\033[00m\\]\\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\$ '" >> ~/.bashrc && \
    echo "CLICOLOR=1" >> ~/.bashrc && \
    echo "LSCOLORS=GxFxCxDxBxegedabagaced" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "## ROS" >> ~/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/acados/lib' >> ~/.bashrc && \
    echo 'export ACADOS_SOURCE_DIR=${HOME}/acados' >> ~/.bashrc && \
    echo 'export PYTHONPATH=${HOME}/acados/interfaces/acados_template:$PYTHONPATH' >> ~/.bashrc && \
    echo 'alias gb="vim ~/.bashrc"'    >> ~/.bashrc && \
    echo 'alias gi="source ~/.bashrc"' >> ~/.bashrc && \
    # Helpful in some X/Qt setups inside containers (prevents aborts in gzclient)
    echo 'export QT_XCB_GL_INTEGRATION=none' >> ~/.bashrc && \
    echo 'export MESA_GL_VERSION_OVERRIDE=3.3' >> ~/.bashrc

# disable Rviz warning
ENV DISABLE_ROS1_EOL_WARNINGS=1

WORKDIR $HOME
