/**
 * @file talker.cpp
 * @author Madhu Narra Chittibabu (mnarrach@umd.edu)
 * @brief this file contains implmentations of Talker
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/pub_sub/talker.h"


Talker::Talker(const std::string &node_name, std::string topic_name,
               int interval)
    : Node(node_name) {
  publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>(topic_name, 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(interval),
                                   std::bind(&Talker::timer_callback, this));
}

void Talker::timer_callback() {
  // Storing the message
  auto message = tutorial_interfaces::msg::Num();
  message.num = 5;  

  // Loggin the message in the terminal
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
  // Publishing the message
  publisher_->publish(message);
}
