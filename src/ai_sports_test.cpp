// Copyright (c) 2023-223 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <memory>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/sport_manager.hpp"
#include "protocol/msg/sport_counts_result.hpp"

using SportSrv = protocol::srv::SportManager;
using SportMsg = protocol::msg::SportCountsResult;
rclcpp::Node::SharedPtr node{nullptr};


void Sport_Result_Callback(SportMsg msg)
{
  if (msg.counts > 0) {
    RCLCPP_INFO(
      node->get_logger(), "algo state:%d, sport_counts: %d, sport_type: %d.",
      msg.algo_switch, msg.counts, msg.sport_type);
  } else {
    RCLCPP_INFO(
      node->get_logger(), "algo state:%d.", msg.algo_switch);
  }
}

void Run(rclcpp::Node::SharedPtr node)
{
  rclcpp::spin(node);
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("ai_sports_test");
  RCLCPP_INFO(node->get_logger(), "Create ai__sports_test node.");
  std::thread process(&Run, node);
  auto ai_sports_client_ = node->create_client<SportSrv>("sport_manager");
  while (!ai_sports_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  // 运动计数
  RCLCPP_INFO(node->get_logger(), "ai sport coming.");
  auto ai_sports_request = std::make_shared<SportSrv::Request>();
  ai_sports_request->sport_type = SportSrv::Request::SPORT_SQUAT;
  ai_sports_request->command = true;
  ai_sports_request->counts = 5;
  ai_sports_request->timeout = 10;
  auto ai_sports_response = ai_sports_client_->async_send_request(ai_sports_request);
  if (ai_sports_response.wait_for(std::chrono::seconds(10)) !=
    std::future_status::ready)
  {
    RCLCPP_ERROR(node->get_logger(), "Request to ai sport service failed.");
    return 1;
  }
  auto ai_sport_sub_ = node->create_subscription<SportMsg>(
    "sport_counts_msg",
    10, std::bind(Sport_Result_Callback, std::placeholders::_1));
  if (process.joinable()) {
    process.join();
  }
  return 0;
}
