cmake_minimum_required(VERSION 3.8)
project(pub_sub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(msg_files
  "msg/String.msg"
)
set(srv_files
  "srv/AddTwoStrings.srv"
  )
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

#Talker
include_directories(include/pub_sub)
add_executable(publisher app/publisher.cpp src/talker.cpp)
ament_target_dependencies(publisher rclcpp std_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
publisher
DESTINATION lib/${PROJECT_NAME})

#Listener 
add_executable(subscriber app/subscriber.cpp src/listener.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs )

install(TARGETS
subscriber
DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(publisher "${cpp_typesupport_target}")
target_link_libraries(subscriber "${cpp_typesupport_target}")

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Add a gtest executable
set(TESTFILES
  test/service_test.cpp)
ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs geometry_msgs rclcpp tf2_ros tf2)
# add_dependencies(${PROJECT_NAME}_test publisher)
target_link_libraries(${PROJECT_NAME}_test "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
