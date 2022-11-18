/**
 * @file publisher.cpp
 * @author Madhu Narra Chittibabu (mnarrach@umd.edu)
 * @brief main file to start the publisher
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <memory>

#include "../include/pub_sub/talker.h"

int main(int argc, char* argv[]) {
  // Initializing the rclcpp
  rclcpp::init(argc, argv);
  int interval =1;
  if (argc<2){
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "rate was not entered, setting rate to default value (1)");
  }
  else{
    interval = atoi(argv[1]);
  }

  // Instantiating and spinning the node
  rclcpp::spin(std::make_shared<Talker>("pub", "messages", interval));

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
