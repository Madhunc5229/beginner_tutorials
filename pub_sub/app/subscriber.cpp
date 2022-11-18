/**
 * @file subscriber.cpp
 * @author Madhu Narra Chittibabu (mnarrach@umd.edu)
 * @brief main file to start the subscriber
 * @version 0.1
 * @date 2022-11-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <memory>

#include "../include/pub_sub/listener.h"

int main(int argc, char* argv[]) {
  // Initializing the rclcpp
  rclcpp::init(argc, argv);

  // Instantiating and spinning the node
  rclcpp::spin(std::make_shared<Listener>("sub", "messages"));

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
