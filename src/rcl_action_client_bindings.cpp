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
#include <rcl_action/action_client.h>
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
        env, action_client, node_handle, [node, env](void* ptr) {
          rcl_action_client_t* action_client =
              reinterpret_cast<rcl_action_client_t*>(ptr);
          rcl_ret_t ret = rcl_action_client_fini(action_client, node);
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

Napi::Value GetNumEntities(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());

  size_t num_subscriptions = 0u;
  size_t num_guard_conditions = 0u;
  size_t num_timers = 0u;
  size_t num_clients = 0u;
  size_t num_services = 0u;

  rcl_ret_t ret;
  ret = rcl_action_client_wait_set_get_num_entities(
      action_client, &num_subscriptions, &num_guard_conditions, &num_timers,
      &num_clients, &num_services);
  if (RCL_RET_OK != ret) {
    rcl_reset_error();
    std::string error_text{
        "Failed to get number of entities for 'rcl_action_client_t'"};
    Napi::Error::New(env, error_text).ThrowAsJavaScriptException();
    return env.Undefined();
  }
  Napi::Object entities = Napi::Object::New(env);
  entities.Set("subscriptionsNumber",
               Napi::Number::New(env, num_subscriptions));
  entities.Set("guardConditionsNumber",
               Napi::Number::New(env, num_guard_conditions));
  entities.Set("timersNumber", Napi::Number::New(env, num_timers));
  entities.Set("clientsNumber", Napi::Number::New(env, num_clients));
  entities.Set("servicesNumber", Napi::Number::New(env, num_services));
  return entities;
}

Napi::Value ActionSendCancelRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  int64_t sequence_number;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_cancel_request(action_client, buffer, &sequence_number),
      RCL_RET_OK, rcl_get_error_string().str);

  return Napi::Number::New(env, static_cast<int32_t>(sequence_number));
}

#if ROS_VERSION >= 2505  // ROS2 >= Kilted
Napi::Value ConfigureActionClientIntrospection(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  RclHandle* node_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      RclHandle::Unwrap(info[2].As<Napi::Object>())->ptr());

  std::string action_name = info[3].As<Napi::String>().Utf8Value();
  std::string package_name = info[4].As<Napi::String>().Utf8Value();
  const rosidl_action_type_support_t* ts =
      GetActionTypeSupport(package_name, action_name);
  rcl_ret_t ret = RCL_RET_ERROR;
  if (ts) {
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    auto qos_profile = GetQoSProfile(info[5]);
    if (qos_profile) {
      publisher_ops.qos = *qos_profile;
    }
    rcl_service_introspection_state_t state =
        static_cast<rcl_service_introspection_state_t>(
            info[6].As<Napi::Number>().Uint32Value());
    ret = rcl_action_client_configure_action_introspection(
        action_client, node, clock, ts, publisher_ops, state);
    if (ret == RCL_RET_OK) {
      return env.Undefined();
    }
  }

  Napi::Error::New(env, "failed to configure action client introspection")
      .ThrowAsJavaScriptException();
  return env.Undefined();
}
#endif  // ROS_VERSION >= 2505

Napi::Object InitActionClientBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("actionCreateClient",
              Napi::Function::New(env, ActionCreateClient));
  exports.Set("actionServerIsAvailable",
              Napi::Function::New(env, ActionServerIsAvailable));
  exports.Set("actionSendGoalRequest",
              Napi::Function::New(env, ActionSendGoalRequest));
  exports.Set("actionSendResultRequest",
              Napi::Function::New(env, ActionSendResultRequest));
  exports.Set("actionTakeFeedback",
              Napi::Function::New(env, ActionTakeFeedback));
  exports.Set("actionTakeStatus", Napi::Function::New(env, ActionTakeStatus));
  exports.Set("getNumEntities", Napi::Function::New(env, GetNumEntities));
  exports.Set("actionSendCancelRequest",
              Napi::Function::New(env, ActionSendCancelRequest));
#if ROS_VERSION >= 2505  // ROS2 >= Kilted
  exports.Set("configureActionClientIntrospection",
              Napi::Function::New(env, ConfigureActionClientIntrospection));
#endif  // ROS_VERSION >= 2505
  return exports;
}

}  // namespace rclnodejs
