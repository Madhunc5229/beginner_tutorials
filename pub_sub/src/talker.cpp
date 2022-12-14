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

#include "talker.h"

Talker::Talker(const std::string &node_name, std::string topic_name,
               int interval)
    : Node(node_name) {
  if (rcutils_logging_set_logger_level(
          this->get_name(), RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) ==
      RCUTILS_RET_OK)
    RCLCPP_DEBUG(this->get_logger(), "Started with DEBUG");
  else
    RCLCPP_INFO(this->get_logger(), "Started without DEBUG");

  // using RCLCPP_FATAL_STREAM to inform about the interval
  if (interval < 1) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                        "Publishing interval is too small!");
  }
  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(interval),
                                   std::bind(&Talker::timer_callback, this));
  service_ = this->create_service<pub_sub::srv::AddTwoStrings>(
      "add_two_strings",
      std::bind(&Talker::addStrings, this, std::placeholders::_1,
                std::placeholders::_2));

  // Using RCLCPP_INFO_STREAM_ONCE to inform about the message that is set
  RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"), "Setting the message");
  message_.data = "Publishing TF between 'world & 'talk'";

  tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  timer_frame_ = this->create_wall_timer(
      std::chrono::seconds(2), std::bind(&Talker::make_transforms, this));
  // Publish static transforms once at startup
}

void Talker::timer_callback() {
  // Loggin the message in the terminal
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s' ", message_.data.c_str());
  // Publishing the message
  publisher_->publish(message_);
}

void Talker::addStrings(
    const std::shared_ptr<pub_sub::srv::AddTwoStrings::Request> request,
    std::shared_ptr<pub_sub::srv::AddTwoStrings::Response> response) {
  response->concatenated = request->text1 + request->text2;

  // using RCLCPP_WARN_STREAM to log the warning that the requesting is coming
  RCLCPP_WARN_STREAM(
      rclcpp::get_logger("rclcpp"),
      "Incoming request\ntext1: " << request->text1 << " "
                                  << "text2: " << request->text2);

  // using RCLCPP_DEBUG_STREAM to log the information about the respone from
  // service
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                      "Sending back response: " << response->concatenated);

  message_.data = response->concatenated;
}

void Talker::make_transforms() {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "talk";

  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;

  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_static_broadcaster_->sendTransform(t);
}
