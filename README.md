<<<<<<< HEAD
# kachaka_develop
Package for running kachaka on ros2
=======
# Kachaka ROS2 Lecture

This is a lecture/educational repository that demonstrates various ROS2 functionalities with the Kachaka robot.

More lecture information: https://mertcookimg.github.io/ros2_lecture/

## Prerequisites
- ROS2 Humble
- Kachaka robot with API enabled

## Packages Overview

### Core Kachaka Packages from [Kachaka API](https://github.com/pf-robotics/kachaka-api)
These packages are provided by the official Kachaka API repository:
- `kachaka_description`([Original Repository](https://github.com/pf-robotics/kachaka-api)): Robot description and URDF files
- `kachaka_interfaces`([Original Repository](https://github.com/pf-robotics/kachaka-api)): ROS2 message and service definitions
- `kachaka_nav2_bringup`([Original Repository](https://github.com/pf-robotics/kachaka-api)): Navigation2 configuration and launch files
- `kachaka_speak`([Original Repository](https://github.com/pf-robotics/kachaka-api)): ROS2 speech synthesis package

### Control Packages
Custom packages developed for this lecture:
- `kachaka_feedforward_control`: Feedforward control implementation
- `kachaka_feedback_control`: Feedback control implementation
- `kachaka_lidar_control`: LiDAR sensor control and processing

### Image Processing Packages
Custom packages developed for this lecture:
- `image_gray_processor`: Grayscale image processing
- `image_edge_detection`: Edge detection algorithms
- `image_yolo_detection`: YOLO-based object detection

### Teleoperation Packages
- `teleop_twist_keyboard` ([Original Repository](https://github.com/ros2/teleop_twist_keyboard)): Keyboard teleoperation node for ROS2

### Integration Packages
- `kachaka_speak_detection`: Speech detection and recognition package

## License Information

| Package Category | License Type | Description |
|-----------------|--------------|-------------|
| Kachaka Packages | Apache License 2.0 | Core Kachaka packages from official API |
| Control Packages | Apache License 2.0 | Custom control packages for this lecture |
| Image Processing | Apache License 2.0 | Custom image processing packages |
| Teleoperation | BSD License 2.0 | teleop_twist_keyboard |
| Integration | Apache License 2.0 | Custom integration packages |

For detailed license information and copyright notices, please refer to individual package directories.

## Building and Usage

1. Make sure you have ROS2 installed
2. Clone this repository
3. Build the workspace

## How to Use

More lecture information: https://mertcookimg.github.io/ros2_lecture/

## Acknowledgments

- [Preferred Robotics, Inc.](https://github.com/pf-robotics) for the Kachaka robot and API
- ROS2 community for the teleop_twist_keyboard package

>>>>>>> 6db67e8 (first commit)
