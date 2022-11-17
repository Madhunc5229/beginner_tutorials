/**
 * @file talker.h
 * @author Madhu Narra Chittibabu (mnarrach@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef PUB_SUB_INCLUDE_PUB_SUB_TALKER_H_
#define PUB_SUB_INCLUDE_PUB_SUB_TALKER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Talker : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Talker object.
   *
   * @param node_name The name of the node which publishes the message.
   * @param interval The interval (in seconds) that publishes the messages.
   * @param topic_name The name of the topic through which the messages have to
   * transported.
   */
  Talker(const std::string &node_name ,
         std::string topic_name, int interval);

 private:
  rclcpp::TimerBase::SharedPtr
      timer_;  //!< The pointer that points to the callback.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  //!< The pointer to the publisher topic.

  /**
   * @brief The callback funtion that prints and publishes a message in the
   * topic.
   *
   */
  void timer_callback();
};  // Talker

#endif  // PUB_SUB_INCLUDE_PUB_SUB_TALKER_H_
