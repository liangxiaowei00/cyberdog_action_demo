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
#include <malloc.h>
#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_action_sample/cyberdog_action_sample.hpp"


namespace cyberdog
{
namespace interaction
{

gesture_sample::gesture_sample(const std::string & name)
{
  this->node_ptr_ = rclcpp::Node::make_shared(name);
  INFO("Creating cyberdog_hand_action object(node)");
}
gesture_sample::~gesture_sample()
{
  INFO("Destroy [cyberdog_hand_action] object(node) begin");
}

bool gesture_sample::Init()
{
  INFO("gesture_sample node init");
  this->action_client_ = this->node_ptr_->create_client<GestureActionSrv>(
    "gesture_action_control");
  this->action_sub_ = this->node_ptr_->create_subscription<GestureActionMsg>(
    "gesture_action_msg", rclcpp::SystemDefaultsQoS(),
    std::bind(&gesture_sample::gesture_action_callback, this, std::placeholders::_1));
  return true;
}


void gesture_sample::gesture_action_callback(const GestureActionMsg::SharedPtr msg)
{
  INFO("gesture_action_callback");
  switch (Gesture_cut(msg->id)) {
    case Gesture_cut::Thumb_Up:
      INFO("get Thumb_Up gesture");
      break;
    case Gesture_cut::Sliding_Hand_Two_Fingers_Up:
      INFO("get Sliding_Hand_Two_Fingers_Up gesture");
      break;
    case Gesture_cut::Pushing_Hand_Or_Two_Fingers_Away:
      INFO("get Pushing_Hand_Or_Two_Fingers_Away gesture");
      break;
    case Gesture_cut::Sliding_Hand_Two_Fingers_Left:
      INFO("get Sliding_Hand_Two_Fingers_Left gesture");
      break;
    case Gesture_cut::Sliding_Hand_Two_Fingers_Right:
      INFO("get Sliding_Hand_Two_Fingers_Right gesture");
      break;
    case Gesture_cut::Stop_Sign:
      INFO("get Stop_Sign gesture");
      break;
    case Gesture_cut::Zooming_In_With_Hand_Or_Two_Fingers:
      INFO("get Zooming_In_With_Hand_Or_Two_Fingers gesture");
      break;
    case Gesture_cut::Zooming_Out_With_Hand_Or_Two_Fingers:
      INFO("get Zooming_Out_With_Hand_Or_Two_Fingers gesture");
      break;
    default:
      INFO("other gesture");
      break;
  }
}


void gesture_sample::Request_Service()
{
// request gesture_action service
  std::shared_ptr<GestureActionSrv::Request> request =
    std::make_shared<GestureActionSrv::Request>();
  request->command = GestureActionSrv::Request::START_ALGO;
  request->timeout = GestureActionSrv::Request::DEFAUT_TIMEOUT;
  while (!this->action_client_->wait_for_service(std::chrono::seconds(100))) {
    if (!rclcpp::ok()) {
      ERROR("Interrupted while waiting for the service. Exiting.");
    }
    INFO("service not available, waiting again...");
  }
  INFO("send request");
  auto result = this->action_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(20)) != std::future_status::ready) {
    ERROR("failed to receive action response");
  } else {
    INFO("got result ...");
  }
}

void gesture_sample::Run()
{
  INFO("cyberdog_hand_action node spin,wait for request");
  rclcpp::spin(this->node_ptr_);
  INFO("cyberdog_hand_action node rclcpp close");
  rclcpp::shutdown();
}

}  // namespace interaction
}  // namespace cyberdog
