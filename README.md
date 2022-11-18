[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
# beginner_tutorials  
## This repository contains simple ROS2 publisher with service that can be called and subscriber code written in C++  

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
## Launching the package
- Launch the nodes with interval parameter which is publishing rate for the publisher node
```
ros2 launch pub_sub pub_sub_launch interval:=1
```
## Service Call
- Open a new terminal, source both ros2 and the package. Then type the following command
- The following service call will take two strings as requests and send a concatenated string as response
- You can check in the terminal where the package was launched that the publishing message will change after the servive call
```
source install/setup.bash
ros2 service call /add_two_strings pub_sub/srv/AddTwoStrings "{text1: "hello", text2: "world"}"
```
## RQT console
- Open a new terminal, source both ros2 and the package. Then type the following command
```
source install/setup.bash
ros2 run rqt_console rqt_console
```
<img src="rqt_log_level.png" width="500" alt="Alt text" title="">
