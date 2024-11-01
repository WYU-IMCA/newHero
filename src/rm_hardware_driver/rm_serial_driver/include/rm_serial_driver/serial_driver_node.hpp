// Created by Chengfu Zou on 2023.7.1
// Copyright (C) FYT Vision Group. All rights reserved.
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

#ifndef SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
#define SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_

// std
#include <memory>
#include <thread>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "rm_utils/heartbeat.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/protocol_factory.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "std_msgs/msg/string.hpp"

namespace imca::serial_driver
{

  // Node wrapper for SerialDriver
  // Implementing secondary development through the Protocol class
  class SerialDriverNode : public rclcpp::Node
  {
    using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

  public:
    explicit SerialDriverNode(const rclcpp::NodeOptions &options);

    ~SerialDriverNode();

    void listenLoop();

    void init();

    // Param client to set detect_color
    struct SetModeClient
    {
      SetModeClient(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr p) : ptr(p) {}
      std::atomic<bool> on_waiting = false;
      std::atomic<int> mode = 0;
      rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr ptr;
    };

    struct SetParamsClient
    {

      SetParamsClient(rclcpp::AsyncParametersClient::SharedPtr p) : ptr(p) {}
      ResultFuturePtr set_param_future;
      // std::atomic<bool> on_waiting = false;
      // std::atomic<int> enemy_color = 0;
      bool on_waiting = false;
      int enemy_color = 0;
      rclcpp::AsyncParametersClient::SharedPtr ptr;
    };
    std::unordered_map<std::string, SetModeClient> set_mode_clients_;
    std::vector<SetParamsClient> set_params_clients_;
    void setMode(SetModeClient &client, const uint8_t mode);

  private:
    // Heartbeat
    HeartBeatPublisher::SharedPtr heartbeat_;

    std::unique_ptr<std::thread> listen_thread_;
    // Protocol
    std::unique_ptr<protocol::Protocol> protocol_;

    std::string target_frame_;

    // rclcpp::TimerBase::SharedPtr timer_;
    //  Subscriptions
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    // Publisher
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_pub_;

    // Broadcast tf from odom to gimbal_link
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    uint8_t previous_receive_color_ = 0;

    bool initial_set_param_ = false;
    double is_reverse_ = 30.0;

    void setParam(SetParamsClient &client, const rclcpp::Parameter &param);

    // bool record_is_open_;
  };

} // namespace imca::serial_driver

#endif // SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
