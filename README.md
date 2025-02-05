# Science System

## Introduction

Controls the life sciences module of the rover.

## Quick Start

Make sure you have the following installed on your (Ubuntu) system:

-   Modern Python 3
-   ROS2 Humble
-   System dependencies: ros-humble-rosbridge-server
-   Python dependencies: pigpio

Then, run the following commands inside the root of the repository:

```bash
# Make sure you have the ROS2 environment sourced

# Build the packages (first time only)
colcon build --symlink-install

# Source the overlay workspace (every time you open a new terminal)
source install/local_setup.bash
```

To run just the systems, run the following command:

```bash
ros2 launch launch/launch.py
```

To run both systems and rosbridge (for testing independently of the rest of the rover), run the following command:

```bash
ros2 launch launch/launch_rosbridge.py
```
