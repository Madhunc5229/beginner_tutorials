/**
 * @file listener.h
 * @author Madhu Narra Chittibabu (mnarrach@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-11-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PUB_SUB_INCLUDE_PUB_SUB_LISTENER_H_
#define PUB_SUB_INCLUDE_PUB_SUB_LISTENER_H_
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "pub_sub/msg/string.hpp"

/**
 * @brief this class contains methods & attributes of a subscriber
 *
 */
class Listener : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Listener object.
   *
   * @param node_name The name of the node which publishes the message.
   * @param topic_name The name of the topic through which the messages have to
   * transported.
   *
   */
  Listener(const std::string &node_name, std::string topic_name);

 private:
  /**
   * @brief A callback to read the message in the topic.
   *
   * @param msg The message that is read from the topic.
   */
  void topic_callback(const std_msgs::msg::String &msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  //!< The pointer that subscribes to the topic.
};                    // Listener

#endif  //  PUB_SUB_INCLUDE_PUB_SUB_LISTENER_H_
