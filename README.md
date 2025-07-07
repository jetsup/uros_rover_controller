# Micro ROS Rover Controller

This package provides a controller for a rover using ROS 2 and micro-ROS. It includes functionality for controlling the rover's movement, speed, and other features.

## Features

- Control rover movement (forward, backward, left, right)
- Adjust speed and turning rates
- Hoot functionality

## Setup and Installation

1. Navigate to your ROS 2 workspace following the instructions from their [official documentation](https://docs.ros.org/en/jazzy/Tutorials/Workspace/Creating-A-Workspace.html).

2. Clone the repository into your workspace's `src` directory:

   ```bash
   git clone https://github.com/jetsup/uros_rover_controller.git 
   ```

3. Build the package:

   ```bash
   colcon build --symlink-install
   ```

4. Source the workspace:

   ```bash
   source install/setup.bash # or source install/setup.zsh
   ```

5. Run the controller node:

   ```bash
   ros2 run uros_rover_controller rover_controller
   ```

## Fixes and Features

- [X] Handle right-left keyboard input for turning
