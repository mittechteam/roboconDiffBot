# Differential Drive based Robot for Robocon 2023

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
<!-- ROS2 Badge -->
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://index.ros.org/doc/ros2/Releases/Release-Humble-Hawksbill/)
<!-- Raspberry pi Badge -->
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-4B-red)](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/)


## Table of Contents
- [Introduction](#introduction)
- [Hardware](#hardware)
- [Software](#software)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)


## Introduction
RoboconDiffBot is a differential drive based robot. It is a part of the Robocon 2023 project. It is a 2 wheeled robot with a differential drive system. It is controlled by a Raspberry Pi 4B and a L298N motor driver.

## Hardware
- Raspberry Pi 4B
- L298N Motor Driver
- 2 DC Motors
- 2 Wheels
- 2 Encoders
- 1 Battery
- 1 IMU

## Software
- ROS2 Humble
- Python3
- C++
- MicroROS

## Usage
- Clone the repository
- Run the following commands
```
cd ~/ros2_ws
colcon build --symlink-install
source install/local_setup.bash
ros2 launch robocon_diff_bot robocon_diff_bot.launch.py
```

## Contributing
- Fork the repository
- Clone the repository
- Create a new branch
- Make changes
- Commit and push
- Create a pull request

## License
[MIT](https://choosealicense.com/licenses/mit/)