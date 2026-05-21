# VBT-MPC: Vision-Based Tactile MPC for Contour Following

This repository contains the simulation and control code associated with the paper **"VBT-MPC: Vision-Based Tactile MPC for Contour Following"**.

Preprint version: [Arxiv](https://arxiv.org/pdf/2605.20392)

IEEE RA-L version: [Coming soon](https://arxiv.org/pdf/2605.20392)

The current branch provides a ROS Noetic/Gazebo simulation environment for contour-following experiments using a Kinova Gen3 robotic arm equipped with a simulated GelSight Mini tactile sensor. The system includes the simulated robot, tactile sensor, object models, feature extraction nodes, and visual-servoing controllers for edge-tracking experiments.

A ROS 2 implementation is maintained in a separate branch.

![Kinova Gen3 and GelSight Mini simulation](images/simulation_gazebo_rviz.png)

---

## Overview

The goal of this repository is to provide the code required to reproduce and extend the visual-tactile contour-following experiments presented in the paper.

The simulator includes:

- A Kinova Gen3 robotic arm model in Gazebo.
- A simulated GelSight Mini tactile sensor mounted on the robot end-effector.
- A free-floating GelSight simulation mode for isolated sensor tests.
- Several object models for contour-following experiments.
- Visual-tactile feature extraction from simulated depth images.
- Multiple visual-servoing controllers for edge tracking.

The repository is intended to support reproducibility of the paper experiments and to provide a baseline for future development of visual-tactile control strategies.

---

## Repository branches

| Branch | Description |
| --- | --- |
| `main` | ROS Noetic simulation environment with Gazebo, Kinova Gen3, and GelSight Mini. |
| `ros2` | ROS 2 implementation of the visual-tactile control framework.  Coming soon|

> **Note:** The `main` branch focuses on the simulator used for the paper experiments. The ROS 2 implementation is maintained separately to keep both environments isolated and easier to reproduce.

---

## Requirements

The recommended way to use this repository is through the provided Visual Studio Code Dev Container.

Main requirements:

- Ubuntu 20.04 or compatible Linux distribution.
- Docker.
- Visual Studio Code.
- Dev Containers extension for VS Code.
- NVIDIA Container Toolkit, only if GPU acceleration is required.

The Dev Container sets up the ROS Noetic environment and the required dependencies automatically.

---

## Installation using Dev Container

Clone the repository:

```bash
git clone https://github.com/EPVelasco/gs_kinova_devcontainer.git
cd gs_kinova_devcontainer
code .
```

When Visual Studio Code opens the repository, select:

```text
Reopen in Container
```

If the prompt does not appear, open the command palette with `Ctrl + Shift + P` and select:

```text
Dev Containers: Reopen in Container
```

VS Code will build the container using the configuration stored in:

```text
.devcontainer/devcontainer.json
```

Once the container is running, open a terminal inside VS Code and compile the Catkin workspace:

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

---

## Running without GPU support

If the computer does not have an NVIDIA GPU, or if NVIDIA dependency errors appear during container creation, use the CPU-only Dev Container configuration.

Replace the contents of:

```text
.devcontainer/devcontainer.json
```

with the contents of:

```text
.devcontainer/devcontainer_w-o_GPU.json
```

Then reopen the repository in the Dev Container.

---

## Quick start

After compiling the workspace, launch the main simulation environment:

```bash
roslaunch kortex_gazebo spawn_gs_Kinova.launch \
  start_rviz:=true \
  use_trajectory_controller:=false \
  gs_sim:=true \
  object_name:=aluminum_profile \
  rviz_config:=test_02.rviz
```

This command launches Gazebo, spawns the Kinova Gen3 robot with the simulated GelSight Mini sensor, loads the selected object, and starts RViz with the requested configuration.

---

## Simulation modes

### 1. Kinova Gen3 with GelSight Mini

Use this mode to reproduce the robot-based contour-following experiments.

```bash
roslaunch kortex_gazebo spawn_gs_Kinova.launch \
  start_rviz:=true \
  use_trajectory_controller:=false \
  gs_sim:=true \
  object_name:=aluminum_profile \
  rviz_config:=test_02.rviz
```

The argument `use_trajectory_controller:=false` disables the default trajectory controller, allowing the robot to be controlled through the visual-servoing velocity controller.

---

### 2. Shoe object simulation

To launch the simulation with the shoe object, use the following object pose:

```bash
roslaunch kortex_gazebo spawn_gs_Kinova.launch \
  start_rviz:=true \
  use_trajectory_controller:=false \
  gs_sim:=true \
  object_name:=shoe \
  x:=0.50 \
  y:=0.0 \
  z:=0.0582 \
  roll:=0.012055 \
  pitch:=0.035225 \
  yaw:=0.0
```

These pose parameters place the shoe object in the correct position and orientation for the contour-following experiment.

---

### 3. Free-floating GelSight Mini sensor

The repository also includes a free-floating GelSight simulation mode. This is useful for testing the tactile sensor model, feature extraction, and low-level velocity commands without the Kinova robot.

```bash
roslaunch gelsight_gazebo start_gazebo_gs.launch
```

This launch file spawns the GelSight sensor as a body with full 6-DOF motion in Gazebo.

Velocity commands can be sent using `geometry_msgs/Twist` messages to:

```text
/cmd_vel
```

The velocity commands are interpreted in the `base_link` frame, which is aligned with the `gs_camera_pcloud` frame. Therefore, linear and angular velocities are applied relative to the current sensor orientation.

---

## Launch arguments

The following arguments can be used with the simulation launch files.

| Argument | Default | Description |
| --- | --- | --- |
| `object_name` | `aluminum_profile` | Name of the STL object loaded in the Gazebo world. |
| `x`, `y`, `z` | `0.5`, `0.5`, `0.0` | Position of the object in the world frame. |
| `roll`, `pitch`, `yaw` | `0.0`, `0.0`, `0.0` | Orientation of the object in RPY angles. |
| `start_rviz` | `true` | Launch RViz automatically. |
| `gs_sim` | `true` | Start the GelSight simulation, feature extractor, and related control nodes. |
| `paused` | `false` | Start Gazebo in paused mode. |
| `gui` | `true` | Show the Gazebo graphical interface. |
| `debug` | `false` | Launch Gazebo with debugger support. |
| `joystick_control_val` | `false` | Launch the joystick control node. A physical joystick is required. |
| `keyboard_control_val` | `false` | Launch the keyboard control node. The `pygame` library is required. |

Example with custom object pose:

```bash
roslaunch gelsight_gazebo start_gazebo_gs.launch \
  object_name:=shoe \
  x:=0.50 \
  y:=0.0 \
  z:=0.0582 \
  roll:=0.012055 \
  pitch:=0.035225 \
  yaw:=0.0
```

---

## Available objects

The `object_name` argument selects the object to be spawned in front of the robot or tactile sensor.

| Object name | Description |
| --- | --- |
| `aluminum_profile` | Long aluminum profile. Useful for straight-edge tracking experiments. |
| `curve1` | Curved object for testing contour-following behavior. |
| `curve2` | Alternative curved object for testing contour-following behavior. |
| `shoe` | Shoe-like object. Requires the specific pose parameters shown above. |

---

## Running the Kinova twist controller

Before running the visual-servoing controllers, launch the simulator and then start the Kinova twist controller:

```bash
cd /home/catkin_ws/src/kinova_dq_nmpc/scripts
python3 kinova_control_twist.py
```

This script enables Cartesian velocity control of the Kinova Gen3 end-effector.

> **Important:** Keep this node running while executing the visual-servoing controllers. Stop it only when setting the robot initial pose manually.

---

## Setting the initial robot pose

To move the robot to the initial pose, stop `kinova_control_twist.py` and run:

```bash
cd /home/catkin_ws/src/kinova_dq_nmpc/scripts
python3 set_kinova_initial.py
```

After the robot reaches the initial pose, start `kinova_control_twist.py` again before running any contour-following controller.

---

## Running the visual-servoing controllers

The repository includes three visual-servoing controllers used to evaluate different contour-following strategies.

Go to the controller scripts folder:

```bash
cd /home/catkin_ws/src/kinova_dq_nmpc/scripts
```

Then run one of the following controllers.

### Classic visual servoing

```bash
python3 classic_visual_servoing.py
```

This controller implements a conventional visual-servoing strategy for edge tracking.

### Decoupled visual servoing

```bash
python3 decoupled_visual_servoing.py
```

This controller separates the control objectives into decoupled components for contour tracking.

### NMPC visual servoing

```bash
python3 nmpc_visual_servoing.py
```

This controller implements the nonlinear model predictive control strategy used for visual-tactile contour following.

> **Note:** Run only one visual-servoing controller at a time.

---

## Suggested execution order

A typical experiment can be launched as follows.

### Terminal 1: launch the simulator

```bash
cd /home/catkin_ws
source devel/setup.bash
roslaunch kortex_gazebo spawn_gs_Kinova.launch \
  start_rviz:=true \
  use_trajectory_controller:=false \
  gs_sim:=true \
  object_name:=aluminum_profile
```

### Terminal 2: start the Kinova twist controller

```bash
cd /home/catkin_ws
source devel/setup.bash
cd /home/catkin_ws/src/kinova_dq_nmpc/scripts
python3 kinova_control_twist.py
```

### Terminal 3: run one visual-servoing controller

```bash
cd /home/catkin_ws
source devel/setup.bash
cd /home/catkin_ws/src/kinova_dq_nmpc/scripts
python3 nmpc_visual_servoing.py
```

---

## Reproducing the paper experiments

The paper experiments can be reproduced by selecting the corresponding object model and controller.

| Experiment type | Object | Suggested controller |
| --- | --- | --- |
| Straight edge tracking | `aluminum_profile` | `classic_visual_servoing.py`, `decoupled_visual_servoing.py`, or `nmpc_visual_servoing.py` |

For direct comparison between controllers, launch the same object and initial pose, then run each controller separately.

---

## Troubleshooting

### The container fails because of NVIDIA dependencies

Use the CPU-only Dev Container configuration:

```text
.devcontainer/devcontainer_w-o_GPU.json
```

Copy its contents into:

```text
.devcontainer/devcontainer.json
```

Then reopen the folder in the Dev Container.

### The robot does not move

Check that `kinova_control_twist.py` is running before launching the visual-servoing controller.

### Multiple controllers are sending commands

Only one controller should publish commands at a time. Stop the currently running controller before launching another one.

---

## Citation

If you use this repository in your research, please cite the associated paper:

```bibtex
@article{velasco2026vbtmpc,
    title={VBT-MPC: Vision-Based Tactile MPC for Contour Following},
    author={Velasco-S{\'a}nchez, Edison and Recalde, Luis F and Guanrui, Li and Gil, Pablo},
    journal={IEEE Robotics and Automation Letters},
    year={2026},
    publisher={IEEE},
    note      = {To be updated after publication}
}
```

---

## Acknowledgements

This repository builds on and acknowledges the following open-source projects:

- [Kinova ROS Kortex](https://github.com/Kinovarobotics/ros_kortex)
- [gelsight_simulation](https://github.com/danfergo/gelsight_simulation)

---

## License

Please refer to the repository license file for usage and distribution terms.
