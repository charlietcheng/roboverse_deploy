# Development Container for Roboverse

This directory contains configuration files for VS Code's Development Container feature, allowing you to develop inside a consistent container environment with pre-installed tools and dependencies.

## Getting Started

1. Install the "Remote - Containers" extension in VS Code
2. Open this project in VS Code
3. When prompted, click "Reopen in Container" or use the command palette (F1) to run "Remote-Containers: Reopen in Container"
4. Wait for the container to build (this might take a few minutes the first time)
5. Once complete, you'll be working inside the development container

## Features

- ROS Noetic with desktop-full installation
- Franka Emika libraries pre-configured
- Python environment with required dependencies
- C++ development tools
- X11 forwarding for GUI applications

## Usage

The development container provides the same environment as the regular Docker setup, but with added development tools and VS Code integration. You can:

- Edit code with full IDE features
- Build the project using catkin
- Run commands in the integrated terminal
- Debug applications
- Use VS Code extensions

## Running Components

To start specific components (like roscore, controllers, etc.), use the terminal in VS Code to run the commands manually:

```bash
# Start roscore
source /catkin_ws/devel/setup.bash
roscore

# In another terminal, start the controller
source /catkin_ws/devel/setup.bash
roslaunch --wait serl_franka_controllers impedance.launch robot_ip:=169.254.67.230 load_gripper:=true
```

Alternatively, you can still use the docker-compose commands from the main project, which will start the services as defined in the root docker-compose.yml file. 