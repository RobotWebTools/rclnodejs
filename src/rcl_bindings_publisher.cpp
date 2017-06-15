// Copyright (c) 2017 Intel Corporation. All rights reserved.
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


#include "rcl_bindings.hpp"

#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/rcl.h>

#include <dlfcn.h>

#include <string>

// TODO(Kenny): remove these temp headers
#include <rosidl_generator_c/string_functions.h>  // NOLINT
#include <std_msgs/msg/string.h>  // NOLINT

#include "handle_manager.hpp"
#include "shadow_node.hpp"
#include "rcl_handle.hpp"

extern "C" {
  typedef const rosidl_message_type_support_t* (*MSG_TYPE_SUPPORT_CALLBACK)();
}


namespace rclnodejs {

NAN_METHOD(CreatePublisher) {
  // Extract arguments
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
      info[0]->ToObject())->GetPtr());
  std::string packageName(*Nan::Utf8String(info[1]->ToString()));
  std::string messageSubFolder(*Nan::Utf8String(info[2]->ToString()));
  std::string messageName(*Nan::Utf8String(info[3]->ToString()));
  std::string topic(*Nan::Utf8String(info[4]->ToString()));

  // Prepare publisher object
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      malloc(sizeof(rcl_publisher_t)));
  *publisher = rcl_get_zero_initialized_publisher();

  // Get type support object dynamically
  // First, the library name
  // TODO(Kenny): support *.dll/etc. on other platforms
  std::string lib_name = "lib";
  lib_name += packageName;
  lib_name += "__rosidl_typesupport_c.so";
  void* lib = dlopen(lib_name.c_str(), RTLD_NOW|RTLD_GLOBAL);
  // User-friendly error message if wrong library name
  std::string lib_error_message = "Cannot load dynamic library (lib='";
  lib_error_message += lib_name;
  lib_error_message += "'). Check spelling or run rosidl_generator_c.";
  RCLN_THROW_EQUAL(lib, nullptr, lib_error_message.c_str());
  // Second, the function name
  MSG_TYPE_SUPPORT_CALLBACK function_ptr = nullptr;
  std::string function_name = RCLN_GET_MSG_TYPE_SUPPORT(packageName,
      messageSubFolder, messageName);
  function_ptr = (MSG_TYPE_SUPPORT_CALLBACK)dlsym(lib, function_name.c_str());
  // User-friendly error message if wrong function name
  std::string function_error_message;
  function_error_message +=
      "Cannot create message type support object from rcl (symbol='";
  function_error_message += function_name;
  function_error_message += "'). Check spelling or run rosidl_generator_c.";
  RCLN_THROW_EQUAL(function_ptr, nullptr, function_error_message.c_str());
  // Now call the function and get value
  const rosidl_message_type_support_t * ts = function_ptr();

  // Using default options
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  // Initialize the publisher
  RCLN_CHECK_AND_THROW(rcl_publisher_init(publisher,
      node, ts, topic.c_str(), &publisher_ops), RCL_RET_OK);

  // Wrap the handle into JS object
  auto newObj = rclnodejs::RclHandle::NewInstance(publisher);

  // Everything is done
  info.GetReturnValue().Set(newObj);
}

NAN_METHOD(rcl_publish_std_string_message) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
      info[0]->ToObject())->GetPtr());

  std_msgs__msg__String msg;
  std_msgs__msg__String__init(&msg);
  rosidl_generator_c__String__assign(&msg.data,
      *Nan::Utf8String(info[1]->ToString()));

  RCLN_CHECK_AND_THROW(rcl_publish(publisher, &msg), RCL_RET_OK);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(PublishMessage) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
      info[0]->ToObject())->GetPtr());

  void* buffer = node::Buffer::Data(info[1]->ToObject());
  // auto size = node::Buffer::Length(info[1]->ToObject());

  RCLN_CHECK_AND_THROW(rcl_publish(publisher, buffer), RCL_RET_OK);

  info.GetReturnValue().Set(Nan::Undefined());
}

}  // namespace rclnodejs
