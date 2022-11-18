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
  publisher_ = this->create_publisher<pub_sub::msg::String>(topic_name, 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(interval), std::bind(&Talker::timer_callback, this));
  service_ = this->create_service<pub_sub::srv::AddTwoStrings>("add_two_strings", std::bind(&Talker::addStrings, this, std::placeholders::_1, std::placeholders::_2));
  message_.text = "hello";
}

void Talker::timer_callback() {
  // Storing the message
 
  // Loggin the message in the terminal
  RCLCPP_INFO(this->get_logger(),"Publishing: text1: '%s' ", message_.text.c_str());
  // Publishing the message
  publisher_->publish(message_);
}

void Talker::addStrings(const std::shared_ptr<pub_sub::srv::AddTwoStrings::Request> request, std::shared_ptr<pub_sub::srv::AddTwoStrings::Response> response){
  response->concatenated = request->text1 + request->text2;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\ntext1: '%s'" " text2: '%s'",  
  //               request->text1, request->text2);                                         
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: '%s'", response->concatenated);
  message_.text = response->concatenated;
}