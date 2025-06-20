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

#include "rcl_serialization_bindings.h"

#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <string>

#include "rcl_utilities.h"

namespace {

struct SerializedMessage {
  explicit SerializedMessage(Napi::Env env, rcutils_allocator_t allocator)
      : env(env) {
    rcl_msg = rmw_get_zero_initialized_serialized_message();
    rcutils_ret_t rcutils_ret =
        rmw_serialized_message_init(&rcl_msg, 0u, &allocator);
    if (RCUTILS_RET_OK != rcutils_ret) {
      Napi::Error::New(env, "failed to initialize serialized message")
          .ThrowAsJavaScriptException();
    }
  }

  ~SerializedMessage() {
    rcutils_ret_t ret = rmw_serialized_message_fini(&rcl_msg);
    if (RCUTILS_RET_OK != ret) {
      Napi::Error::New(env,
                       "failed to fini rcl_serialized_msg_t in destructor.")
          .ThrowAsJavaScriptException();
      rcutils_reset_error();
    }
  }

  rcl_serialized_message_t rcl_msg;
  Napi::Env env;
};

}  // namespace

namespace rclnodejs {

Napi::Value Serialize(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string package_name = info[0].As<Napi::String>().Utf8Value();
  std::string message_sub_folder = info[1].As<Napi::String>().Utf8Value();
  std::string message_name = info[2].As<Napi::String>().Utf8Value();
  void* ros_msg = info[3].As<Napi::Buffer<char>>().Data();
  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);

  // Create a serialized message object.
  SerializedMessage serialized_msg(env, rcutils_get_default_allocator());

  rmw_ret_t rmw_ret = rmw_serialize(ros_msg, ts, &serialized_msg.rcl_msg);
  if (RMW_RET_OK != rmw_ret) {
    Napi::Error::New(env, "Failed to serialize ROS message")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }
  Napi::Buffer<char> buffer = Napi::Buffer<char>::Copy(
      env, reinterpret_cast<const char*>(serialized_msg.rcl_msg.buffer),
      serialized_msg.rcl_msg.buffer_length);
  return buffer;
}

Napi::Value Deserialize(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string package_name = info[0].As<Napi::String>().Utf8Value();
  std::string message_sub_folder = info[1].As<Napi::String>().Utf8Value();
  std::string message_name = info[2].As<Napi::String>().Utf8Value();
  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);
  Napi::Buffer<char> serialized = info[3].As<Napi::Buffer<char>>();
  void* msg_taken = info[4].As<Napi::Buffer<char>>().Data();

  // Create a serialized message object.
  rcl_serialized_message_t serialized_msg =
      rmw_get_zero_initialized_serialized_message();
  serialized_msg.buffer_capacity = serialized.Length();
  serialized_msg.buffer_length = serialized.Length();
  serialized_msg.buffer = reinterpret_cast<uint8_t*>(serialized.Data());

  rmw_ret_t rmw_ret = rmw_deserialize(&serialized_msg, ts, msg_taken);

  if (RMW_RET_OK != rmw_ret) {
    Napi::Error::New(env, "failed to deserialize ROS message")
        .ThrowAsJavaScriptException();
  }

  return env.Undefined();
}

Napi::Object InitSerializationBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("serialize", Napi::Function::New(env, Serialize));
  exports.Set("deserialize", Napi::Function::New(env, Deserialize));
  return exports;
}

}  // namespace rclnodejs
