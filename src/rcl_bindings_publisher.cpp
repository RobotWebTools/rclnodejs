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

#include "handle_manager.hpp"
#include "shadow_node.hpp"
#include "rcl_handle.hpp"

// TODO: remove these 3 temp header
#include <std_msgs/msg/string.h>
#include <rosidl_generator_c/message_type_support_struct.h>
#include <rosidl_generator_c/string_functions.h>

namespace rclnodejs {

NAN_METHOD(CreatePublisher) {
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
      info[0]->ToObject())->GetPtr());

  std::string topic(*Nan::Utf8String(info[2]->ToString()));

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      malloc(sizeof(rcl_publisher_t)));

  *publisher = rcl_get_zero_initialized_publisher();

  // TODO(Kenny): remove this and use the generic ts from args
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  RCLN_CHECK_AND_THROW(rcl_publisher_init(publisher,
      node, ts, topic.c_str(), &publisher_ops));

  auto newObj = rclnodejs::RclHandle::NewInstance(publisher);

  info.GetReturnValue().Set(newObj);
}

NAN_METHOD(rcl_publish_std_string_message) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
      info[0]->ToObject())->GetPtr());

  std_msgs__msg__String msg;
  std_msgs__msg__String__init(&msg);
  rosidl_generator_c__String__assign(&msg.data, *Nan::Utf8String(info[1]->ToString()));

  RCLN_CHECK_AND_THROW(rcl_publish(publisher, &msg));

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(PublishMessage) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
      info[0]->ToObject())->GetPtr());

  void* buffer = node::Buffer::Data(info[1]->ToObject());
  // auto size = node::Buffer::Length(info[1]->ToObject());

  RCLN_CHECK_AND_THROW(rcl_publish(publisher, buffer));

  info.GetReturnValue().Set(Nan::Undefined());
}

}
