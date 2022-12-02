#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pub_sub/talker.h"


rclcpp::init(argc, argv);
int interval = 1;
if (argc < 2) {
RCLCPP_WARN_STREAM(
    rclcpp::get_logger("rclcpp"),
    "rate was not entered, setting rate to default value (1)");
} else {
interval = atoi(argv[1]);
}

// Instantiating and spinning the node
rclcpp::spin(std::make_shared<Talker>("pub", "messages", interval));

// Shutdown
rclcpp::shutdown();

return 0;

/**
 * @brief main function to run all tests
 * 
 */
int main(int argc,
         char **argv) {
  ros::init(argc, argv, "change_output_string");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}