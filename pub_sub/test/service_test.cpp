/**
 * @file service_test.cpp
 * @author Madhu Narra Chittibabu (mnarrach@umd.edu)
 * @brief this file tests the service present in the publisher
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <chrono>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

#include "pub_sub/srv/add_two_strings.hpp"


class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override { std::cout << "DONE WITH TEARDOWN" << std::endl; }

 protected:
  rclcpp::Node::SharedPtr node_;
};


TEST(TaskPlanningFixture, addString) {
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("add_two_strings_client");
  rclcpp::Client<pub_sub::srv::AddTwoStrings>::SharedPtr client =
      node->create_client<pub_sub::srv::AddTwoStrings>("add_two_strings");
  bool exists(client->wait_for_service(std::chrono::milliseconds(20000)));
  std::cout << "CLIENT EXISTS TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(exists);
  std::cout << "CLIENT EXISTS TEST PASSED!!" << std::endl;

  auto request = std::make_shared<pub_sub::srv::AddTwoStrings::Request>();
  request->text1 = "Hello";
  request->text2 = " World";

  auto result = client->async_send_request(request);
  auto ret = rclcpp::spin_until_future_complete(
      node, result, std::chrono::milliseconds(500));  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  std::cout << "SERVICE RESPONSE OUTPUT TEST BEGINNING!!" << std::endl;
  EXPECT_EQ("Hello World", result.get()->concatenated);
  std::cout << "SERVICE RESPONSE OUTPUT TEST PASSED!!" << std::endl;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
