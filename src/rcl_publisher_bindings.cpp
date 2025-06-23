// Copyright (c) 2025, The Robot Web Tools Contributors
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

#include "rcl_publisher_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value CreatePublisher(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  std::string package_name = info[1].As<Napi::String>().Utf8Value();
  std::string message_sub_folder = info[2].As<Napi::String>().Utf8Value();
  std::string message_name = info[3].As<Napi::String>().Utf8Value();
  std::string topic = info[4].As<Napi::String>().Utf8Value();

  rcl_publisher_t* publisher =
      reinterpret_cast<rcl_publisher_t*>(malloc(sizeof(rcl_publisher_t)));
  *publisher = rcl_get_zero_initialized_publisher();

  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);

  if (ts) {
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    auto qos_profile = GetQoSProfile(info[5]);

    if (qos_profile) {
      publisher_ops.qos = *qos_profile;
    }

    THROW_ERROR_IF_NOT_EQUAL(
        rcl_publisher_init(publisher, node, ts, topic.c_str(), &publisher_ops),
        RCL_RET_OK, rcl_get_error_string().str);

    auto js_obj = RclHandle::NewInstance(
        env, publisher, node_handle, [node, env](void* ptr) {
          rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(ptr);
          rcl_ret_t ret = rcl_publisher_fini(publisher, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                             rcl_get_error_string().str);
        });

    return js_obj;
  } else {
    Napi::Error::New(env, GetErrorMessageAndClear())
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }
}

Napi::Value Publish(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  void* buffer = info[1].As<Napi::Buffer<char>>().Data();
  THROW_ERROR_IF_NOT_EQUAL(rcl_publish(publisher, buffer, nullptr), RCL_RET_OK,
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value GetPublisherTopic(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* topic = rcl_publisher_get_topic_name(publisher);
  return Napi::String::New(env, topic);
}

Napi::Value PublishRawMessage(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  auto object = info[1].As<Napi::Buffer<char>>();
  rcl_serialized_message_t serialized_msg =
      rmw_get_zero_initialized_serialized_message();
  serialized_msg.buffer_capacity = object.Length();
  serialized_msg.buffer_length = serialized_msg.buffer_capacity;
  serialized_msg.buffer = reinterpret_cast<uint8_t*>(object.Data());

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_publish_serialized_message(publisher, &serialized_msg, nullptr),
      RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value GetSubscriptionCount(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_publisher_get_subscription_count(publisher, &count), RCL_RET_OK,
      rcl_get_error_string().str);

  return Napi::Number::New(env, count);
}

Napi::Value WaitForAllAcked(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  bool lossless;
  int64_t nanoseconds = info[1].As<Napi::BigInt>().Int64Value(&lossless);

  rcl_ret_t ret = rcl_publisher_wait_for_all_acked(publisher, nanoseconds);
  if (RCL_RET_OK == ret) {
    return Napi::Boolean::New(env, true);
  } else if (RCL_RET_TIMEOUT == ret) {
    return Napi::Boolean::New(env, false);
  }
  Napi::Error::New(env, "Failed to wait for all acknowledgements")
      .ThrowAsJavaScriptException();
  return env.Undefined();
}

Napi::Object InitPublisherBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createPublisher", Napi::Function::New(env, CreatePublisher));
  exports.Set("publish", Napi::Function::New(env, Publish));
  exports.Set("getPublisherTopic", Napi::Function::New(env, GetPublisherTopic));
  exports.Set("publishRawMessage", Napi::Function::New(env, PublishRawMessage));
  exports.Set("getSubscriptionCount",
              Napi::Function::New(env, GetSubscriptionCount));
  exports.Set("waitForAllAcked", Napi::Function::New(env, WaitForAllAcked));
  return exports;
}

}  // namespace rclnodejs
