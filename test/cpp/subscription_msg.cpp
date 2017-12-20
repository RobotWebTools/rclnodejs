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
#include <iomanip>
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

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr byte_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr char_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int8_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr uint8_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr int16_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr uint16_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int32_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr uint32_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr int64_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr uint64_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float32_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr float64_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr color_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr array_pub = nullptr;
rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub  = nullptr;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub = nullptr;

void boolCallback(std_msgs::msg::Bool::SharedPtr msg) {
  bool_pub->publish(msg);
}

void byteCallback(std_msgs::msg::Byte::SharedPtr msg) {
  byte_pub->publish(msg);
}

void charCallback(std_msgs::msg::Char::SharedPtr msg) {
  char_pub->publish(msg);
}

void strCallback(std_msgs::msg::String::SharedPtr msg) {
  str_pub->publish(msg);
}

void int8Callback(std_msgs::msg::Int8::SharedPtr msg) {
  int8_pub->publish(msg);
}

void uint8Callback(std_msgs::msg::UInt8::SharedPtr msg) {
  uint8_pub->publish(msg);
}

void int16Callback(std_msgs::msg::Int16::SharedPtr msg) {
  int16_pub->publish(msg);
}

void uint16Callback(std_msgs::msg::UInt16::SharedPtr msg) {
  uint16_pub->publish(msg);
}

void int32Callback(std_msgs::msg::Int32::SharedPtr msg) {
  int32_pub->publish(msg);
}

void uint32Callback(std_msgs::msg::UInt32::SharedPtr msg) {
  uint32_pub->publish(msg);
}

void int64Callback(std_msgs::msg::Int64::SharedPtr msg) {
  int64_pub->publish(msg);
}

void uint64Callback(std_msgs::msg::UInt64::SharedPtr msg) {
  uint64_pub->publish(msg);
}

void float32Callback(std_msgs::msg::Float32::SharedPtr msg) {
  float32_pub->publish(msg);
}

void float64Callback(std_msgs::msg::Float64::SharedPtr msg) {
  float64_pub->publish(msg);
}

void colorCallback(std_msgs::msg::ColorRGBA::SharedPtr msg) {
  color_pub->publish(msg);
}

void arrayCallback(std_msgs::msg::ByteMultiArray::SharedPtr msg) {
  array_pub->publish(msg);
}

void headerCallback(std_msgs::msg::Header::SharedPtr msg) {
  header_pub->publish(msg);
}

void jointstateCallback(sensor_msgs::msg::JointState::SharedPtr msg) {
  jointstate_pub->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cpp_subscription");

  // Bool
  bool_pub = node->create_publisher<std_msgs::msg::Bool>(
    std::string("back_") + std::string("Bool_js_cpp_channel"), rmw_qos_profile_default);
  auto bool_sub = node->create_subscription<std_msgs::msg::Bool>(std::string("Bool_js_cpp_channel"),
    boolCallback, rmw_qos_profile_default);

  // Byte
  byte_pub = node->create_publisher<std_msgs::msg::Byte>(
    std::string("back_") + std::string("Byte_js_cpp_channel"), rmw_qos_profile_default);
  auto byte_sub = node->create_subscription<std_msgs::msg::Byte>(std::string("Byte_js_cpp_channel"),
    byteCallback, rmw_qos_profile_default);

  // Char
  char_pub = node->create_publisher<std_msgs::msg::Char>(
    std::string("back_") + std::string("Char_js_cpp_channel"), rmw_qos_profile_default);
  auto char_sub = node->create_subscription<std_msgs::msg::Char>(std::string("Char_js_cpp_channel"),
    charCallback, rmw_qos_profile_default);

  // String
  str_pub = node->create_publisher<std_msgs::msg::String>(
    std::string("back_") + std::string("String_js_cpp_channel"), rmw_qos_profile_default);
  auto str_sub = node->create_subscription<std_msgs::msg::String>(std::string("String_js_cpp_channel"),
    strCallback, rmw_qos_profile_default);

  // Int8
  int8_pub = node->create_publisher<std_msgs::msg::Int8>(
    std::string("back_") + std::string("Int8_js_cpp_channel"), rmw_qos_profile_default);
  auto int8_sub = node->create_subscription<std_msgs::msg::Int8>(std::string("Int8_js_cpp_channel"),
    int8Callback, rmw_qos_profile_default);

  // UInt8
  uint8_pub = node->create_publisher<std_msgs::msg::UInt8>(
    std::string("back_") + std::string("UInt8_js_cpp_channel"), rmw_qos_profile_default);
  auto uint8_sub = node->create_subscription<std_msgs::msg::UInt8>(std::string("UInt8_js_cpp_channel"),
    uint8Callback, rmw_qos_profile_default);

  // Int16
  int16_pub = node->create_publisher<std_msgs::msg::Int16>(
    std::string("back_") + std::string("Int16_js_cpp_channel"), rmw_qos_profile_default);
  auto int16_sub = node->create_subscription<std_msgs::msg::Int16>(std::string("Int16_js_cpp_channel"),
    int16Callback, rmw_qos_profile_default);

  // UInt16
  uint16_pub = node->create_publisher<std_msgs::msg::UInt16>(
    std::string("back_") + std::string("UInt16_js_cpp_channel"), rmw_qos_profile_default);
  auto uint16_sub = node->create_subscription<std_msgs::msg::UInt16>(std::string("UInt16_js_cpp_channel"),
    uint16Callback, rmw_qos_profile_default);
  
  // Int32
  int32_pub = node->create_publisher<std_msgs::msg::Int32>(
    std::string("back_") + std::string("Int32_js_cpp_channel"), rmw_qos_profile_default);
  auto int32_sub = node->create_subscription<std_msgs::msg::Int32>(std::string("Int32_js_cpp_channel"),
    int32Callback, rmw_qos_profile_default);

  // UInt32
  uint32_pub = node->create_publisher<std_msgs::msg::UInt32>(
    std::string("back_") + std::string("UInt32_js_cpp_channel"), rmw_qos_profile_default);
  auto uint32_sub = node->create_subscription<std_msgs::msg::UInt32>(std::string("UInt32_js_cpp_channel"),
    uint32Callback, rmw_qos_profile_default);
  
  // Int64
  int64_pub = node->create_publisher<std_msgs::msg::Int64>(
    std::string("back_") + std::string("Int64_js_cpp_channel"), rmw_qos_profile_default);
  auto int64_sub = node->create_subscription<std_msgs::msg::Int64>(std::string("Int64_js_cpp_channel"),
    int64Callback, rmw_qos_profile_default);

  // UInt64
  uint64_pub = node->create_publisher<std_msgs::msg::UInt64>(
    std::string("back_") + std::string("UInt64_js_cpp_channel"), rmw_qos_profile_default);
  auto uint64_sub = node->create_subscription<std_msgs::msg::UInt64>(std::string("UInt64_js_cpp_channel"),
    uint64Callback, rmw_qos_profile_default);

  // Float32
  float32_pub = node->create_publisher<std_msgs::msg::Float32>(
    std::string("back_") + std::string("Float32_js_cpp_channel"), rmw_qos_profile_default);
  auto float32_sub = node->create_subscription<std_msgs::msg::Float32>(std::string("Float32_js_cpp_channel"),
    float32Callback, rmw_qos_profile_default);

  // Float64
  float64_pub = node->create_publisher<std_msgs::msg::Float64>(
    std::string("back_") + std::string("Float64_js_cpp_channel"), rmw_qos_profile_default);
  auto float64_sub = node->create_subscription<std_msgs::msg::Float64>(std::string("Float64_js_cpp_channel"),
    float64Callback, rmw_qos_profile_default);
  
  // ColorRGBA
  color_pub = node->create_publisher<std_msgs::msg::ColorRGBA>(
    std::string("back_") + std::string("ColorRGBA_js_cpp_channel"), rmw_qos_profile_default);
  auto colorrgba_sub = node->create_subscription<std_msgs::msg::ColorRGBA>(std::string("ColorRGBA_js_cpp_channel"),
    colorCallback, rmw_qos_profile_default);
  
  // Array
  array_pub = node->create_publisher<std_msgs::msg::ByteMultiArray>(
    std::string("back_") + std::string("ByteMultiArray_js_cpp_channel"), rmw_qos_profile_default);
  auto array_sub = node->create_subscription<std_msgs::msg::ByteMultiArray>(std::string("Array_js_cpp_channel"),
    arrayCallback, rmw_qos_profile_default);

  // Header
  header_pub = node->create_publisher<std_msgs::msg::Header>(
    std::string("back_") + std::string("Header_js_cpp_channel"), rmw_qos_profile_default);
  auto header_sub = node->create_subscription<std_msgs::msg::Header>(std::string("Header_js_cpp_channel"),
    headerCallback, rmw_qos_profile_default);

  // Complex object: JointState
  jointstate_pub = node->create_publisher<sensor_msgs::msg::JointState>(
    std::string("back_") + std::string("JointState_js_cpp_channel"), rmw_qos_profile_default);
  auto jointstate_sub = node->create_subscription<sensor_msgs::msg::JointState>(std::string("JointState_js_cpp_channel"),
    jointstateCallback, rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
