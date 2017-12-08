// Copyright (c) 2017 Intel Corporation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

void print_usage() {
  std::cout << "Usage for publisher_msg executable:" << std::endl
            << "publisher_msg -t [topic_name] -m [msg_type] -h" << std::endl
            << "options:" << std::endl
            << "-h : Print this help function." << std::endl
            << "-t topic_name : Specify the topic on which to publish. Defaults to chatter." << std::endl
            << "-m msg_type : Specify the message type to publish. Defaults to String." << std::endl;
}

int main(int argc, char* argv[]) {
  std::map<std::string, int> msg_type_map = {
    {std::string("Bool"), 1},
    {std::string("Byte"), 2},
    {std::string("Char"), 3},
    {std::string("String"), 4},
    {std::string("Int8"), 5},
    {std::string("UInt8"), 6},
    {std::string("Int16"), 7},
    {std::string("UInt16"), 8},
    {std::string("Int32"), 9},
    {std::string("UInt32"), 10},
    {std::string("Int64"), 11},
    {std::string("UInt64"), 12},
    {std::string("Float32"), 13},
    {std::string("Float64"), 14},
    {std::string("ColorRGBA"), 15},
    {std::string("Array"), 16},
    {std::string("Header"), 17},
    {std::string("JointState"), 18}
  };

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("cpp_publisher");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }
  
  auto topic = std::string("chatter");
  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }
  auto msg_type = std::string("String");
  if (rcutils_cli_option_exist(argv, argv + argc, "-m")) {
    msg_type = std::string(rcutils_cli_get_option(argv, argv + argc, "-m"));
  }

  // Bool
  auto bool_publisher = node->create_publisher<std_msgs::msg::Bool>(topic, custom_qos_profile);
  auto bool_msg = std::make_shared<std_msgs::msg::Bool>();
  bool_msg->data = true;

  // Byte
  auto byte_publisher = node->create_publisher<std_msgs::msg::Byte>(topic, custom_qos_profile);
  auto byte_msg = std::make_shared<std_msgs::msg::Byte>();
  byte_msg->data = 255;

  // Char
  auto char_publisher = node->create_publisher<std_msgs::msg::Char>(topic, custom_qos_profile);
  auto char_msg = std::make_shared<std_msgs::msg::Char>();
  char_msg->data = 'A';

  // String
  auto string_publisher = node->create_publisher<std_msgs::msg::String>(topic, custom_qos_profile);
  auto string_msg = std::make_shared<std_msgs::msg::String>();
  string_msg->data = std::string("Hello World");

  // Int8
  auto int8_publisher = node->create_publisher<std_msgs::msg::Int8>(topic, custom_qos_profile);
  auto int8_msg = std::make_shared<std_msgs::msg::Int8>();
  int8_msg->data = 127;

  // UInt8
  auto uint8_publisher = node->create_publisher<std_msgs::msg::UInt8>(topic, custom_qos_profile);
  auto uint8_msg = std::make_shared<std_msgs::msg::UInt8>();
  uint8_msg->data = 255;

  // Int16
  auto int16_publisher = node->create_publisher<std_msgs::msg::Int16>(topic, custom_qos_profile);
  auto int16_msg = std::make_shared<std_msgs::msg::Int16>();
  int16_msg->data = 0x7fff;

  // UInt16
  auto uint16_publisher = node->create_publisher<std_msgs::msg::UInt16>(topic, custom_qos_profile);
  auto uint16_msg = std::make_shared<std_msgs::msg::UInt16>();
  uint16_msg->data = 0xffff;

  // Int32
  auto int32_publisher = node->create_publisher<std_msgs::msg::Int32>(topic, custom_qos_profile);
  auto int32_msg = std::make_shared<std_msgs::msg::Int32>();
  int32_msg->data = 0x7fffffffL;

  // UInt32
  auto uint32_publisher = node->create_publisher<std_msgs::msg::UInt32>(topic, custom_qos_profile);
  auto uint32_msg = std::make_shared<std_msgs::msg::UInt32>();
  uint32_msg->data = 0xffffffffUL;

  // Int64
  auto int64_publisher = node->create_publisher<std_msgs::msg::Int64>(topic, custom_qos_profile);
  auto int64_msg = std::make_shared<std_msgs::msg::Int64>();
  long long max_js_int = (2LL << 52) - 1;
  int64_msg->data = max_js_int;

  // UInt64
  auto uint64_publisher = node->create_publisher<std_msgs::msg::UInt64>(topic, custom_qos_profile);
  auto uint64_msg = std::make_shared<std_msgs::msg::UInt64>();
  uint64_msg->data = max_js_int;

  // Float32
  auto float32_publisher = node->create_publisher<std_msgs::msg::Float32>(topic, custom_qos_profile);
  auto float32_msg = std::make_shared<std_msgs::msg::Float32>();
  float32_msg->data = 3.14;

  // Float64
  auto float64_publisher = node->create_publisher<std_msgs::msg::Float64>(topic, custom_qos_profile);
  auto float64_msg = std::make_shared<std_msgs::msg::Float64>();
  float64_msg->data = 3.1415926;

  // ColorRGBA
  auto colorrgba_publisher = node->create_publisher<std_msgs::msg::ColorRGBA>(topic, custom_qos_profile);
  auto colorrgba_msg = std::make_shared<std_msgs::msg::ColorRGBA>();
  colorrgba_msg->a = 0.5;
  colorrgba_msg->r = 127;
  colorrgba_msg->g = 255;
  colorrgba_msg->b = 255;

  // Array
  auto array_publisher = node->create_publisher<std_msgs::msg::ByteMultiArray>(topic, custom_qos_profile);
  auto dim = std::make_shared<std_msgs::msg::MultiArrayDimension>();
  dim->label = std::string("length");
  dim->size = 1;
  dim->stride = 3;

  auto layout = std::make_shared<std_msgs::msg::MultiArrayLayout>();
  layout->dim = std::vector<std_msgs::msg::MultiArrayDimension>{*dim};
  layout->data_offset = 0;

  auto array_msg = std::make_shared<std_msgs::msg::ByteMultiArray>();
  array_msg->layout = *layout;
  array_msg->data = std::vector<uint8_t>{65, 66, 67};

  // Header
  auto header_publisher = node->create_publisher<std_msgs::msg::Header>(topic, custom_qos_profile);
  auto time = std::make_shared<builtin_interfaces::msg::Time>();
  time->sec = 123456;
  time->nanosec = 789;

  auto header_msg = std::make_shared<std_msgs::msg::Header>();
  header_msg->stamp = *time;
  header_msg->frame_id = std::string("main frame");

  // Complex object: JointState
  auto jointstate_publisher = node->create_publisher<sensor_msgs::msg::JointState>(topic, custom_qos_profile);
  auto head_time = std::make_shared<builtin_interfaces::msg::Time>();
  head_time->sec = 123456;
  head_time->nanosec = 789;

  auto header = std::make_shared<std_msgs::msg::Header>();
  header->stamp = *head_time;
  header->frame_id = std::string("main frame");

  auto jointstate_msg = std::make_shared<sensor_msgs::msg::JointState>();
  jointstate_msg->header = *header;
  jointstate_msg->name = std::vector<std::string>{"Tom", "Jerry"};
  jointstate_msg->position = std::vector<double>{1.0, 2.0};
  jointstate_msg->velocity = std::vector<double>{2.0, 3.0};
  jointstate_msg->effort = std::vector<double>{4.0, 5.0, 6.0};

  rclcpp::WallRate loop_rate(2);
  while (rclcpp::ok()) {
    // std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    switch (msg_type_map[msg_type]) {
      case 1:
        bool_publisher->publish(bool_msg);
        break;
      case 2:
        byte_publisher->publish(byte_msg);
        break;
      case 3:
        char_publisher->publish(char_msg);
        break;
      case 4:
        string_publisher->publish(string_msg);
        break;
      case 5:
        int8_publisher->publish(int8_msg);
        break;
      case 6:
        uint8_publisher->publish(uint8_msg);
        break;
      case 7:
        int16_publisher->publish(int16_msg);
        break;
      case 8:
        uint16_publisher->publish(uint16_msg);
        break;
      case 9:
        int32_publisher->publish(int32_msg);
        break;
      case 10:
        uint32_publisher->publish(uint32_msg);
        break;
      case 11:
        int64_publisher->publish(int64_msg);
        break;
      case 12:
        uint64_publisher->publish(uint64_msg);
        break;
      case 13:
        float32_publisher->publish(float32_msg);
        break;
      case 14:
        float64_publisher->publish(float64_msg);
        break;
      case 15:
        colorrgba_publisher->publish(colorrgba_msg);
        break;
      case 16:
        array_publisher->publish(array_msg);
        break;
      case 17:
        header_publisher->publish(header_msg);
        break;
      case 18:
        jointstate_publisher->publish(jointstate_msg);
        break;
      default:
        std::cerr << "Unsupported message types" << std::endl;
        break;        
    }    
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
