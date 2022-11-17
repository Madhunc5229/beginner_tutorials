/**
 * @file listener.cpp
 * @author Madhu Narra Chittibabu (mnarrach@umd.edu)
 * @brief this file contains implmentations of Listener
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "../include/pub_sub/listener.h"

Listener::Listener(const std::string &node_name, std::string topic_name)
    : Node(node_name) {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic_name, 10,
      std::bind(&Listener::topic_callback, this, std::placeholders::_1));
}

void Listener::topic_callback(const std_msgs::msg::String &msg) const {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
