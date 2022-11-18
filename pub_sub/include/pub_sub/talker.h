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
#include "pub_sub/msg/string.hpp"
#include "pub_sub/srv/add_two_strings.hpp"

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
  rclcpp::TimerBase::SharedPtr timer_;  //!< The pointer that points to the callback.
  rclcpp::Publisher<pub_sub::msg::String>::SharedPtr publisher_;  //!< The pointer to the publisher topic.
  rclcpp::Service<pub_sub::srv::AddTwoStrings>::SharedPtr service_; //!< The pointer to the service.
  pub_sub::msg::String message_;
  

  /**
   * @brief The callback funtion that prints and publishes a message in the
   * topic.
   *
   */
  void timer_callback();
  /**
   * @brief this function is called when service call is made
   * 
   * @param request input parameters to process the call
   * @param response output of the service call
   */
  void addStrings(const std::shared_ptr<pub_sub::srv::AddTwoStrings::Request> request, std::shared_ptr<pub_sub::srv::AddTwoStrings::Response> response);

};  // Talker

#endif  // PUB_SUB_INCLUDE_PUB_SUB_TALKER_H_
