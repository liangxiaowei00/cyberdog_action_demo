// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef CYBERDOG_ACTION_SAMPLE__CYBERDOG_ACTION_SAMPLE_HPP_
#define CYBERDOG_ACTION_SAMPLE__CYBERDOG_ACTION_SAMPLE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/gesture_action_control.hpp"
#include "protocol/msg/gesture_action_result.hpp"

namespace cyberdog
{
namespace interaction
{

enum class Gesture_cut
{
  No_gesture,
  Pulling_Hand_Or_Two_Fingers_In,
  Pushing_Hand_Or_Two_Fingers_Away,
  Sliding_Hand_Two_Fingers_Up,
  Sliding_Hand_Two_Fingers_Down,
  Sliding_Hand_Two_Fingers_Left,
  Sliding_Hand_Two_Fingers_Right,
  Stop_Sign,
  Thumb_Up,
  Zooming_In_With_Hand_Or_Two_Fingers,
  Zooming_Out_With_Hand_Or_Two_Fingers,
  Thumb_Down
};

class gesture_sample
{
  using GestureActionSrv = protocol::srv::GestureActionControl;
  using GestureActionMsg = protocol::msg::GestureActionResult;

public:
  explicit gesture_sample(const std::string & name);
  bool Init();
  void Run();
  void Request_Service();

  ~gesture_sample();

private:
  void gesture_action_callback(const GestureActionMsg::SharedPtr msg);
  rclcpp::Node::SharedPtr node_ptr_{nullptr};
  rclcpp::executors::MultiThreadedExecutor executor_;

  rclcpp::Subscription<GestureActionMsg>::SharedPtr action_sub_{nullptr};
  rclcpp::Client<GestureActionSrv>::SharedPtr action_client_{nullptr};
};  // class gesture_sample
}    // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_ACTION_SAMPLE__CYBERDOG_ACTION_SAMPLE_HPP_
