[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
# beginner_tutorials
  
## This repository contains simple ROS2 publisher and subscriber code written in C++.

## Dependencies

- Ubuntu 20.04 LTS
- ROS2 Humble
- colcon
- rosdep

## Building the package

### source the ROS2 setup bash.
```
source /opt/ros/humble/setup.bash
```

### Clone the repository
```
cd <ros2 workspace folder>/src
git clone https://github.com/Madhunc5229/beginner_tutorials.git
```


### Build the package using colcon
```
cd ..
colcon build --packages-select pub_sub
```

### Source the package after building
```
source install/setup.bash
```

## Running the publisher
- Open a new terminal, source both ros2 and the package. Then type the following command
```
ros2 run pub_sub publisher
```
## Running the subscriber
- Open a new terminal, source both ros2 and the package. Then type the following command
```
ros2 run pub_sub subscriber
```
