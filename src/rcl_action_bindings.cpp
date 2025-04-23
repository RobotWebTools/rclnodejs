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

#include "rcl_action_bindings.h"

#include <napi.h>
#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rmw/error_handling.h>

#include <memory>
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
        env, action_server, node_handle, [node](void* ptr) {
          rcl_action_server_t* action_server =
              reinterpret_cast<rcl_action_server_t*>(ptr);
          rcl_ret_t ret = rcl_action_server_fini(action_server, node);
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

Napi::Value ActionAcceptNewGoal(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rcl_action_goal_info_t* buffer = reinterpret_cast<rcl_action_goal_info_t*>(
      info[1].As<Napi::Buffer<char>>().Data());

  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(
          malloc(sizeof(rcl_action_goal_handle_t)));

  *goal_handle = *rcl_action_accept_new_goal(action_server, buffer);
  if (!goal_handle) {
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcl_reset_error();
    return env.Undefined();
  }

  auto js_obj =
      RclHandle::NewInstance(env, goal_handle, nullptr, [](void* ptr) {
        rcl_action_goal_handle_t* goal_handle =
            reinterpret_cast<rcl_action_goal_handle_t*>(ptr);
        rcl_ret_t ret = rcl_action_goal_handle_fini(goal_handle);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
      });

  return js_obj;
}

Napi::Value ActionUpdateGoalState(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* goal_handle_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(goal_handle_handle->ptr());
  rcl_action_goal_event_t event = static_cast<rcl_action_goal_event_t>(
      info[1].As<Napi::Number>().Int32Value());

  THROW_ERROR_IF_NOT_EQUAL(rcl_action_update_goal_state(goal_handle, event),
                           RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
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

Napi::Value ActionGoalHandleIsActive(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* goal_handle_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(goal_handle_handle->ptr());

  bool is_active = rcl_action_goal_handle_is_active(goal_handle);

  return Napi::Boolean::New(env, is_active);
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

Napi::Value ActionGoalHandleGetStatus(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* goal_handle_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(goal_handle_handle->ptr());

  rcl_action_goal_state_t status;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_goal_handle_get_status(goal_handle, &status), RCL_RET_OK,
      rcl_get_error_string().str);

  return Napi::Number::New(env, static_cast<int32_t>(status));
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
  auto js_obj =
      RclHandle::NewInstance(env, cancel_response_ptr, nullptr, [](void* ptr) {
        rcl_action_cancel_response_t* cancel_response_ptr =
            reinterpret_cast<rcl_action_cancel_response_t*>(ptr);
        rcl_ret_t ret = rcl_action_cancel_response_fini(cancel_response_ptr);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
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

Napi::Value ActionGetClientNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_action_get_client_names_and_types_by_node(
                               node, &allocator, node_name.c_str(),
                               node_namespace.c_str(), &names_and_types),
                           "Failed to action client names and types.");

  Napi::Array result_list = Napi::Array::New(env, names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  return result_list;
}

Napi::Value ActionGetServerNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_action_get_server_names_and_types_by_node(
                               node, &allocator, node_name.c_str(),
                               node_namespace.c_str(), &names_and_types),
                           "Failed to action server names and types");

  Napi::Array result_list = Napi::Array::New(env, names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  return result_list;
}

Napi::Value ActionGetNamesAndTypes(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_action_get_names_and_types(node, &allocator, &names_and_types),
      "Failed to action server names and types");

  Napi::Array result_list = Napi::Array::New(env, names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  return result_list;
}

Napi::Object InitActionBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("actionCreateClient",
              Napi::Function::New(env, ActionCreateClient));
  exports.Set("actionCreateServer",
              Napi::Function::New(env, ActionCreateServer));
  exports.Set("actionServerIsAvailable",
              Napi::Function::New(env, ActionServerIsAvailable));
  exports.Set("actionSendGoalRequest",
              Napi::Function::New(env, ActionSendGoalRequest));
  exports.Set("actionTakeGoalRequest",
              Napi::Function::New(env, ActionTakeGoalRequest));
  exports.Set("actionSendGoalResponse",
              Napi::Function::New(env, ActionSendGoalResponse));
  exports.Set("actionTakeGoalResponse",
              Napi::Function::New(env, ActionTakeGoalResponse));
  exports.Set("actionSendCancelRequest",
              Napi::Function::New(env, ActionSendCancelRequest));
  exports.Set("actionTakeCancelRequest",
              Napi::Function::New(env, ActionTakeCancelRequest));
  exports.Set("actionSendCancelResponse",
              Napi::Function::New(env, ActionSendCancelResponse));
  exports.Set("actionTakeCancelResponse",
              Napi::Function::New(env, ActionTakeCancelResponse));
  exports.Set("actionSendResultRequest",
              Napi::Function::New(env, ActionSendResultRequest));
  exports.Set("actionTakeResultRequest",
              Napi::Function::New(env, ActionTakeResultRequest));
  exports.Set("actionSendResultResponse",
              Napi::Function::New(env, ActionSendResultResponse));
  exports.Set("actionTakeResultResponse",
              Napi::Function::New(env, ActionTakeResultResponse));
  exports.Set("actionAcceptNewGoal",
              Napi::Function::New(env, ActionAcceptNewGoal));
  exports.Set("actionUpdateGoalState",
              Napi::Function::New(env, ActionUpdateGoalState));
  exports.Set("actionPublishStatus",
              Napi::Function::New(env, ActionPublishStatus));
  exports.Set("actionTakeStatus", Napi::Function::New(env, ActionTakeStatus));
  exports.Set("actionGoalHandleIsActive",
              Napi::Function::New(env, ActionGoalHandleIsActive));
  exports.Set("actionNotifyGoalDone",
              Napi::Function::New(env, ActionNotifyGoalDone));
  exports.Set("actionGoalHandleGetStatus",
              Napi::Function::New(env, ActionGoalHandleGetStatus));
  exports.Set("actionPublishFeedback",
              Napi::Function::New(env, ActionPublishFeedback));
  exports.Set("actionTakeFeedback",
              Napi::Function::New(env, ActionTakeFeedback));
  exports.Set("actionProcessCancelRequest",
              Napi::Function::New(env, ActionProcessCancelRequest));
  exports.Set("actionServerGoalExists",
              Napi::Function::New(env, ActionServerGoalExists));
  exports.Set("actionExpireGoals", Napi::Function::New(env, ActionExpireGoals));
  exports.Set("actionGetClientNamesAndTypesByNode",
              Napi::Function::New(env, ActionGetClientNamesAndTypesByNode));
  exports.Set("actionGetServerNamesAndTypesByNode",
              Napi::Function::New(env, ActionGetServerNamesAndTypesByNode));
  exports.Set("actionGetNamesAndTypes",
              Napi::Function::New(env, ActionGetNamesAndTypes));

  return exports;
}

}  // namespace rclnodejs
