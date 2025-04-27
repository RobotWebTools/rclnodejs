// Copyright (c) 2020 Matt Richard. All rights reserved.
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

#include "rcl_action_client_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>

#include <string>

#include "handle_manager.h"
#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value ActionCreateClient(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string action_name = info[1].As<Napi::String>().Utf8Value();
  std::string interface_name = info[2].As<Napi::String>().Utf8Value();
  std::string package_name = info[3].As<Napi::String>().Utf8Value();

  const rosidl_action_type_support_t* ts =
      GetActionTypeSupport(package_name, interface_name);

  if (ts) {
    rcl_action_client_options_t action_client_ops =
        rcl_action_client_get_default_options();

    auto goal_service_qos = GetQoSProfile(info[4]);
    auto result_service_qos = GetQoSProfile(info[5]);
    auto cancel_service_qos = GetQoSProfile(info[6]);
    auto feedback_topic_qos = GetQoSProfile(info[7]);
    auto status_topic_qos = GetQoSProfile(info[8]);

    if (goal_service_qos) {
      action_client_ops.goal_service_qos = *goal_service_qos;
    }
    if (result_service_qos) {
      action_client_ops.result_service_qos = *result_service_qos;
    }
    if (cancel_service_qos) {
      action_client_ops.cancel_service_qos = *cancel_service_qos;
    }
    if (feedback_topic_qos) {
      action_client_ops.feedback_topic_qos = *feedback_topic_qos;
    }
    if (status_topic_qos) {
      action_client_ops.status_topic_qos = *status_topic_qos;
    }

    rcl_action_client_t* action_client = reinterpret_cast<rcl_action_client_t*>(
        malloc(sizeof(rcl_action_client_t)));
    *action_client = rcl_action_get_zero_initialized_client();

    THROW_ERROR_IF_NOT_EQUAL(
        rcl_action_client_init(action_client, node, ts, action_name.c_str(),
                               &action_client_ops),
        RCL_RET_OK, rcl_get_error_string().str);
    auto js_obj = RclHandle::NewInstance(
        env, action_client, node_handle, [node](void* ptr) {
          rcl_action_client_t* action_client =
              reinterpret_cast<rcl_action_client_t*>(ptr);
          rcl_ret_t ret = rcl_action_client_fini(action_client, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
        });

    return js_obj;
  } else {
    Napi::Error::New(env, GetErrorMessageAndClear())
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }
}

Napi::Value ActionServerIsAvailable(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());

  bool is_available;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_action_server_is_available(node, action_client, &is_available),
      rcl_get_error_string().str);

  return Napi::Boolean::New(env, is_available);
}

Napi::Value ActionSendGoalRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  int64_t sequence_number;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_goal_request(action_client, buffer, &sequence_number),
      RCL_RET_OK, rcl_get_error_string().str);

  return Napi::Number::New(env, static_cast<int32_t>(sequence_number));
}

Napi::Value ActionTakeCancelRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request = info[1].As<Napi::Buffer<char>>().Data();
  rcl_ret_t ret =
      rcl_action_take_cancel_request(action_server, header, taken_request);
  if (ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) {
    auto js_obj = RclHandle::NewInstance(env, header, nullptr,
                                         [](void* ptr) { free(ptr); });
    return js_obj;
  }

  return env.Undefined();
}

Napi::Value ActionSendResultRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  int64_t sequence_number;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_result_request(action_client, buffer, &sequence_number),
      RCL_RET_OK, rcl_get_error_string().str);

  return Napi::Number::New(env, static_cast<int32_t>(sequence_number));
}

Napi::Value ActionTakeFeedback(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  rcl_ret_t ret = rcl_action_take_feedback(action_client, buffer);
  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcl_reset_error();
    return Napi::Boolean::New(env, false);
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    return Napi::Boolean::New(env, true);
  }
  return env.Undefined();
}

Napi::Value ActionTakeStatus(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  rcl_ret_t ret = rcl_action_take_status(action_client, buffer);
  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    rcl_reset_error();
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    return Napi::Boolean::New(env, false);
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    return Napi::Boolean::New(env, true);
  }
  return env.Undefined();
}

Napi::Object InitActionClientBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("actionCreateClient",
              Napi::Function::New(env, ActionCreateClient));
  exports.Set("actionServerIsAvailable",
              Napi::Function::New(env, ActionServerIsAvailable));
  exports.Set("actionSendGoalRequest",
              Napi::Function::New(env, ActionSendGoalRequest));
  exports.Set("actionTakeCancelRequest",
              Napi::Function::New(env, ActionTakeCancelRequest));
  exports.Set("actionSendResultRequest",
              Napi::Function::New(env, ActionSendResultRequest));
  exports.Set("actionTakeFeedback",
              Napi::Function::New(env, ActionTakeFeedback));
  exports.Set("actionTakeStatus", Napi::Function::New(env, ActionTakeStatus));
  return exports;
}

}  // namespace rclnodejs
