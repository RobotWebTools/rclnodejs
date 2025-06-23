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

#include "rcl_action_server_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl_action/action_server.h>
#include <rcl_action/rcl_action.h>

#include <string>

#include "handle_manager.h"
#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value ActionCreateServer(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  RclHandle* clock_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());
  std::string action_name = info[2].As<Napi::String>().Utf8Value();
  std::string interface_name = info[3].As<Napi::String>().Utf8Value();
  std::string package_name = info[4].As<Napi::String>().Utf8Value();
  int64_t result_timeout = info[10].As<Napi::Number>().Int64Value();

  const rosidl_action_type_support_t* ts =
      GetActionTypeSupport(package_name, interface_name);

  if (ts) {
    rcl_action_server_options_t action_server_ops =
        rcl_action_server_get_default_options();

    auto goal_service_qos = GetQoSProfile(info[5]);
    auto result_service_qos = GetQoSProfile(info[6]);
    auto cancel_service_qos = GetQoSProfile(info[7]);
    auto feedback_topic_qos = GetQoSProfile(info[8]);
    auto status_topic_qos = GetQoSProfile(info[9]);

    if (goal_service_qos) {
      action_server_ops.goal_service_qos = *goal_service_qos;
    }
    if (result_service_qos) {
      action_server_ops.result_service_qos = *result_service_qos;
    }
    if (cancel_service_qos) {
      action_server_ops.cancel_service_qos = *cancel_service_qos;
    }
    if (feedback_topic_qos) {
      action_server_ops.feedback_topic_qos = *feedback_topic_qos;
    }
    if (status_topic_qos) {
      action_server_ops.status_topic_qos = *status_topic_qos;
    }

    action_server_ops.result_timeout.nanoseconds =
        static_cast<rcl_duration_value_t>(RCL_S_TO_NS(result_timeout));

    rcl_action_server_t* action_server = reinterpret_cast<rcl_action_server_t*>(
        malloc(sizeof(rcl_action_server_t)));
    *action_server = rcl_action_get_zero_initialized_server();

    THROW_ERROR_IF_NOT_EQUAL(
        rcl_action_server_init(action_server, node, clock, ts,
                               action_name.c_str(), &action_server_ops),
        RCL_RET_OK, rcl_get_error_string().str);
    auto js_obj = RclHandle::NewInstance(
        env, action_server, node_handle, [node, env](void* ptr) {
          rcl_action_server_t* action_server =
              reinterpret_cast<rcl_action_server_t*>(ptr);
          rcl_ret_t ret = rcl_action_server_fini(action_server, node);
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

Napi::Value ActionTakeResultRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());

  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request = info[1].As<Napi::Buffer<char>>().Data();
  rcl_ret_t ret =
      rcl_action_take_result_request(action_server, header, taken_request);
  if (ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) {
    auto js_obj = RclHandle::NewInstance(env, header, nullptr,
                                         [](void* ptr) { free(ptr); });
    return js_obj;
  }

  return env.Undefined();
}

Napi::Value ActionTakeGoalRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request = info[1].As<Napi::Buffer<char>>().Data();
  rcl_ret_t ret =
      rcl_action_take_goal_request(action_server, header, taken_request);
  if (ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) {
    auto js_obj = RclHandle::NewInstance(env, header, nullptr,
                                         [](void* ptr) { free(ptr); });
    return js_obj;
  }

  return env.Undefined();
}

Napi::Value ActionSendGoalResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap(info[1].As<Napi::Object>())->ptr());
  void* buffer = info[2].As<Napi::Buffer<char>>().Data();

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_goal_response(action_server, header, buffer), RCL_RET_OK,
      rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value ActionSendCancelResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap(info[1].As<Napi::Object>())->ptr());
  void* buffer = info[2].As<Napi::Buffer<char>>().Data();

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_cancel_response(action_server, header, buffer),
      RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value ActionSendResultResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap(info[1].As<Napi::Object>())->ptr());
  void* buffer = info[2].As<Napi::Buffer<char>>().Data();

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_result_response(action_server, header, buffer),
      RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value ActionTakeGoalResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  rcl_ret_t ret = rcl_action_take_goal_response(action_client, header, buffer);
  int64_t sequence_number = header->sequence_number;
  free(header);

  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    rcl_reset_error();
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    return Napi::Number::New(env, static_cast<int32_t>(sequence_number));
  }
  return env.Undefined();
}

Napi::Value ActionTakeCancelResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  rcl_ret_t ret =
      rcl_action_take_cancel_response(action_client, header, buffer);
  int64_t sequence_number = header->sequence_number;
  free(header);

  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    rcl_reset_error();
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    return Napi::Number::New(env, static_cast<int32_t>(sequence_number));
  }
  return env.Undefined();
}

Napi::Value ActionTakeResultResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_client_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  rcl_ret_t ret =
      rcl_action_take_result_response(action_client, header, buffer);
  int64_t sequence_number = header->sequence_number;
  free(header);

  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    rcl_reset_error();
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    return Napi::Number::New(env, static_cast<int32_t>(sequence_number));
  }
  return env.Undefined();
}

Napi::Value ActionExpireGoals(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  int64_t max_num_goals = info[1].As<Napi::Number>().Int64Value();
  rcl_action_goal_info_t* buffer = reinterpret_cast<rcl_action_goal_info_t*>(
      info[2].As<Napi::Buffer<char>>().Data());

  size_t num_expired;
  THROW_ERROR_IF_NOT_EQUAL(rcl_action_expire_goals(action_server, buffer,
                                                   max_num_goals, &num_expired),
                           RCL_RET_OK, rcl_get_error_string().str);

  return Napi::Number::New(env, static_cast<int32_t>(num_expired));
}

Napi::Value ActionPublishStatus(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());

  rcl_action_goal_status_array_t status_message =
      rcl_action_get_zero_initialized_goal_status_array();

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_get_goal_status_array(action_server, &status_message),
      RCL_RET_OK, rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_publish_status(action_server, &status_message), RCL_RET_OK,
      rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value ActionNotifyGoalDone(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(rcl_action_notify_goal_done(action_server),
                           RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value ActionPublishFeedback(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_action_server_t* action_server = reinterpret_cast<rcl_action_server_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  THROW_ERROR_IF_NOT_EQUAL(rcl_action_publish_feedback(action_server, buffer),
                           RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value ActionProcessCancelRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();
  rcl_action_cancel_request_t* cancel_request =
      reinterpret_cast<rcl_action_cancel_request_t*>(buffer);
  void* response_buffer = info[2].As<Napi::Buffer<char>>().Data();
  action_msgs__srv__CancelGoal_Response* response =
      reinterpret_cast<action_msgs__srv__CancelGoal_Response*>(response_buffer);

  rcl_action_cancel_response_t* cancel_response_ptr =
      reinterpret_cast<rcl_action_cancel_response_t*>(
          malloc(sizeof(rcl_action_cancel_response_t)));

  *cancel_response_ptr = rcl_action_get_zero_initialized_cancel_response();

  rcl_ret_t ret = rcl_action_process_cancel_request(
      action_server, cancel_request, cancel_response_ptr);
  if (ret != RCL_RET_OK) {
    // fetch the error triggered by rcl_action_process_cancel_request
    rcutils_error_string_t cancel_error = rcl_get_error_string();
    rcl_reset_error();
    rcl_ret_t ret_fini = rcl_action_cancel_response_fini(cancel_response_ptr);
    if (ret_fini != RCL_RET_OK) {
      RCUTILS_LOG_WARN_NAMED(
          PACKAGE_NAME,
          "There was an error finalizing the action cancel response: %s",
          rcl_get_error_string().str);
      rcl_reset_error();
    }
    free(cancel_response_ptr);
    Napi::Error::New(env, cancel_error.str).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  *response = cancel_response_ptr->msg;
  auto js_obj = RclHandle::NewInstance(
      env, cancel_response_ptr, nullptr, [env](void* ptr) {
        rcl_action_cancel_response_t* cancel_response_ptr =
            reinterpret_cast<rcl_action_cancel_response_t*>(ptr);
        rcl_ret_t ret = rcl_action_cancel_response_fini(cancel_response_ptr);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                           rcl_get_error_string().str);
      });
  return js_obj;
}

Napi::Value ActionServerGoalExists(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rcl_action_goal_info_t* buffer = reinterpret_cast<rcl_action_goal_info_t*>(
      info[1].As<Napi::Buffer<char>>().Data());

  bool exists = rcl_action_server_goal_exists(action_server, buffer);

  return Napi::Boolean::New(env, exists);
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

#if ROS_VERSION >= 2505  // ROS2 >= Kilted
Napi::Value ConfigureActionServerIntrospection(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
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
    ret = rcl_action_server_configure_action_introspection(
        action_server, node, clock, ts, publisher_ops, state);
    if (ret == RCL_RET_OK) {
      return env.Undefined();
    }
  }

  Napi::Error::New(env, "failed to configure action server introspection")
      .ThrowAsJavaScriptException();
  return env.Undefined();
}
#endif  // ROS_VERSION >= 2505

Napi::Object InitActionServerBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("actionCreateServer",
              Napi::Function::New(env, ActionCreateServer));
  exports.Set("actionTakeResultRequest",
              Napi::Function::New(env, ActionTakeResultRequest));
  exports.Set("actionTakeGoalRequest",
              Napi::Function::New(env, ActionTakeGoalRequest));
  exports.Set("actionSendGoalResponse",
              Napi::Function::New(env, ActionSendGoalResponse));
  exports.Set("actionSendCancelResponse",
              Napi::Function::New(env, ActionSendCancelResponse));
  exports.Set("actionSendResultResponse",
              Napi::Function::New(env, ActionSendResultResponse));
  exports.Set("actionTakeGoalResponse",
              Napi::Function::New(env, ActionTakeGoalResponse));
  exports.Set("actionTakeCancelResponse",
              Napi::Function::New(env, ActionTakeCancelResponse));
  exports.Set("actionTakeResultResponse",
              Napi::Function::New(env, ActionTakeResultResponse));
  exports.Set("actionExpireGoals", Napi::Function::New(env, ActionExpireGoals));
  exports.Set("actionPublishStatus",
              Napi::Function::New(env, ActionPublishStatus));
  exports.Set("actionNotifyGoalDone",
              Napi::Function::New(env, ActionNotifyGoalDone));
  exports.Set("actionPublishFeedback",
              Napi::Function::New(env, ActionPublishFeedback));
  exports.Set("actionProcessCancelRequest",
              Napi::Function::New(env, ActionProcessCancelRequest));
  exports.Set("actionServerGoalExists",
              Napi::Function::New(env, ActionServerGoalExists));
  exports.Set("actionTakeCancelRequest",
              Napi::Function::New(env, ActionTakeCancelRequest));
#if ROS_VERSION >= 2505  // ROS2 >= Kilted
  exports.Set("configureActionServerIntrospection",
              Napi::Function::New(env, ConfigureActionServerIntrospection));
#endif  // ROS_VERSION >= 2505
  return exports;
}

}  // namespace rclnodejs
