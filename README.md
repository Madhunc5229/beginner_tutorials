[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
# beginner_tutorials  
## This repository contains simple ROS2 publisher with service that can be called and subscriber code written in C++  

## Dependencies

- Ubuntu 20.04 LTS / 22.04 LTS
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
- Launch the nodes with `interval` parameter which is publishing rate for the publisher node and `record` paramter as true if you want a rosbag recorded file for 15 seconds.
- The bag files will be saved in results folder
```
ros2 launch pub_sub pub_sub.launch interval:=1 record:=true
```
- To play the rosbag file (To better vizuale the bag file, run ONLY the subsciber node and play the bag file and verify that the messages are recieved by subscriber)
```
ros2 run pub_sub subscriber
```
```
ros2 bag play src/beginner_tutorials/pub_sub/results/ros_bag_file.bag 
```
## Inspecting tf tree
- Open a new terminal and run the following to generate a frames pdf in results folder which will have information about tf_tree
```
ros2 run tf2_tools view_frames -o src/beginner_tutorials/pub_sub/results/frames_tree/frames
```
## Service Call
- Open a new terminal, source both ros2 and the package. Then type the following command
- The following service call will take two strings as requests and send a concatenated string as response
- You can check in the terminal where the package was launched that the publishing message will change after the servive call
```
source install/setup.bash
ros2 service call /add_two_strings pub_sub/srv/AddTwoStrings "{text1: "hello", text2: "world"}"
```

## Running ROS Tests
- Before running the tests, make sure the publisher is running. If not run the follwing command
```
ros2 run pub_sub publisher
```
- Now run the test cases
```
colcon test --event-handlers console_direct+ --packages-select pub_sub
```

## Running cpplint check
- Run the following command to run cpplint check
```
cpplint --filter=-build/c++11,+build/c++17,-build/name,-build/include_order,-runtime/explicit --recursive pub_sub/. > pub_sub/results/cpplint.txt
```

## Running cppcheck 
- Run the following command to run cppceck
```
cppcheck --enable=all --std=c++17 pub_sub/app/ pub_sub/include/pub_sub/ pub_sub/src pub_sub/test --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > pub_sub/results/cppcheck.txt
```

## RQT console
- Open a new terminal, source both ros2 and the package. Then type the following command
```
source install/setup.bash
ros2 run rqt_console rqt_console
```
<img src="pub_sub/results/rqt_console/rqt_log_level.png" width="800" alt="Alt text" title="">