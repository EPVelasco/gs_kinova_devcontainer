FROM osrf/ros:noetic-desktop-focal

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash


ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    git build-essential ros-noetic-perception-pcl ros-noetic-rviz ros-noetic-eigen-conversions iputils-ping screen vim nano && \
    rm -rf /var/lib/apt/lists/*

# Install Python 3.9 and pip
RUN apt-get update && apt-get install -y \
    python3.9 \
    python3-pip

ENV HOME=/home/${USERNAME}

RUN apt-get update && apt-get install -y \
    libosmesa6-dev \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglfw3 \
    libglfw3-dev \
    libglew-dev \
    patchelf \
    mesa-utils 
    
# Install ros package depends for gs_sensor
RUN apt-get install ros-noetic-effort-controllers* ros-noetic-serial -y
RUN apt-get install python-is-python3 python3-tk -y
RUN pip install scipy

RUN apt update && apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential



RUN apt-get update && apt-get install -y python3 python3-pip
RUN python3 -m pip install conan==1.59

#reactivate ROS key
RUN sed -i '/packages.ros.org/s/^/#/' /etc/apt/sources.list /etc/apt/sources.list.d/*.list || true
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros1-latest-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt-get update && apt-get install -y \
    ros-noetic-ros-controllers \
    ros-noetic-moveit*\
    ros-noetic-ompl 

USER $USERNAME
RUN conan config set general.revisions_enabled=1
RUN conan profile new default --detect > /dev/null
RUN conan profile update settings.compiler.libcxx=libstdc++11 default

WORKDIR $HOME

# --- Build Acados ---
RUN git clone https://github.com/acados/acados.git && \
    cd acados && \
    git checkout 37e17d31890ab54e5a855f1fe787fbf2f5d43bdb && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_INSTALL_DIR="${HOME}/acados" .. && \
    make install -j$(nproc) && \
    cd ${HOME}/acados && make shared_library && make examples_c && \
    pip install -e ${HOME}/acados/interfaces/acados_template

# --- Install Python tools ---
RUN pip install --no-cache-dir pyyaml pynput casadi osqp

# --- Build tera_renderer ---
RUN git clone https://github.com/acados/tera_renderer && \
    cd tera_renderer && \
    curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    /bin/bash -c "source ${HOME}/.cargo/env && cargo build --verbose --release" && \
    cp ${HOME}/tera_renderer/target/release/t_renderer ${HOME}/acados/bin/t_renderer

# bash update,  Acados + tera_renderer env vars ---
RUN echo "TERM=xterm-256color" >> ~/.bashrc && \
    echo "# COLOR Text" >> ~/.bashrc && \
    echo "PS1='\[\033[01;33m\]\u\[\033[01;33m\]@\[\033[01;33m\]\h\[\033[01;34m\]:\[\033[00m\]\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc && \
    echo "CLICOLOR=1" >> ~/.bashrc && \
    echo "LSCOLORS=GxFxCxDxBxegedabagaced" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "## ROS" >> ~/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/acados/lib' >> ~/.bashrc && \
    echo 'export ACADOS_SOURCE_DIR=${HOME}/acados' >> ~/.bashrc && \
    echo 'alias gb="vim ~/.bashrc"'    >> ~/.bashrc && \
    echo 'alias gi="source ~/.bashrc"' >> ~/.bashrc


# disable Rviz warning 
ENV DISABLE_ROS1_EOL_WARNINGS=1
   
WORKDIR ${HOME}
