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

#include "rcl_action_bindings.hpp"

#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rmw/error_handling.h>

#include <memory>
#include <string>
#include <vector>

#include "handle_manager.hpp"
#include "macros.hpp"
#include "rcl_handle.hpp"
#include "rcl_utilities.hpp"

namespace rclnodejs {

NAN_METHOD(ActionCreateClient) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string action_name(
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked()));
  std::string interface_name(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));
  std::string package_name(
      *Nan::Utf8String(info[3]->ToString(currentContent).ToLocalChecked()));

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
    auto js_obj =
        RclHandle::NewInstance(action_client, node_handle, [node](void* ptr) {
          rcl_action_client_t* action_client =
              reinterpret_cast<rcl_action_client_t*>(ptr);
          rcl_ret_t ret = rcl_action_client_fini(action_client, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
        });

    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear().c_str());
  }
}

NAN_METHOD(ActionCreateServer) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  RclHandle* clock_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());
  std::string action_name(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));
  std::string interface_name(
      *Nan::Utf8String(info[3]->ToString(currentContent).ToLocalChecked()));
  std::string package_name(
      *Nan::Utf8String(info[4]->ToString(currentContent).ToLocalChecked()));
  int64_t result_timeout = info[10]->IntegerValue(currentContent).FromJust();

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
    auto js_obj =
        RclHandle::NewInstance(action_server, node_handle, [node](void* ptr) {
          rcl_action_server_t* action_server =
              reinterpret_cast<rcl_action_server_t*>(ptr);
          rcl_ret_t ret = rcl_action_server_fini(action_server, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
        });

    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear().c_str());
  }
}

NAN_METHOD(ActionServerIsAvailable) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());

  bool is_available;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_action_server_is_available(node, action_client, &is_available),
      rcl_get_error_string().str);

  v8::Local<v8::Boolean> result = Nan::New<v8::Boolean>(is_available);
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionSendGoalRequest) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());

  int64_t sequence_number;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_goal_request(action_client, buffer, &sequence_number),
      RCL_RET_OK, rcl_get_error_string().str);

  v8::Local<v8::Integer> result =
      Nan::New<v8::Integer>(static_cast<int32_t>(sequence_number));
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionTakeGoalRequest) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_ret_t ret =
      rcl_action_take_goal_request(action_server, header, taken_request);
  if (ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) {
    auto js_obj =
        RclHandle::NewInstance(header, nullptr, [](void* ptr) { free(ptr); });
    info.GetReturnValue().Set(js_obj);
    return;
  }

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionSendGoalResponse) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[1]).ToLocalChecked())
          ->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[2]).ToLocalChecked());

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_goal_response(action_server, header, buffer), RCL_RET_OK,
      rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionTakeGoalResponse) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  rcl_ret_t ret = rcl_action_take_goal_response(action_client, header, buffer);
  int64_t sequence_number = header->sequence_number;
  free(header);

  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    v8::Local<v8::Integer> result =
        Nan::New<v8::Integer>(static_cast<int32_t>(sequence_number));
    info.GetReturnValue().Set(result);
    return;
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionSendCancelRequest) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());

  int64_t sequence_number;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_cancel_request(action_client, buffer, &sequence_number),
      RCL_RET_OK, rcl_get_error_string().str);

  v8::Local<v8::Integer> result =
      Nan::New<v8::Integer>(static_cast<int32_t>(sequence_number));
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionTakeCancelRequest) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_ret_t ret =
      rcl_action_take_cancel_request(action_server, header, taken_request);
  if (ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) {
    auto js_obj =
        RclHandle::NewInstance(header, nullptr, [](void* ptr) { free(ptr); });
    info.GetReturnValue().Set(js_obj);
    return;
  }

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionSendCancelResponse) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[1]).ToLocalChecked())
          ->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[2]).ToLocalChecked());

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_cancel_response(action_server, header, buffer),
      RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionTakeCancelResponse) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  rcl_ret_t ret =
      rcl_action_take_cancel_response(action_client, header, buffer);
  int64_t sequence_number = header->sequence_number;
  free(header);

  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    v8::Local<v8::Integer> result =
        Nan::New<v8::Integer>(static_cast<int32_t>(sequence_number));
    info.GetReturnValue().Set(result);
    return;
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionSendResultRequest) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());

  int64_t sequence_number;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_result_request(action_client, buffer, &sequence_number),
      RCL_RET_OK, rcl_get_error_string().str);

  v8::Local<v8::Integer> result =
      Nan::New<v8::Integer>(static_cast<int32_t>(sequence_number));
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionTakeResultRequest) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());

  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_ret_t ret =
      rcl_action_take_result_request(action_server, header, taken_request);
  if (ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) {
    auto js_obj =
        RclHandle::NewInstance(header, nullptr, [](void* ptr) { free(ptr); });
    info.GetReturnValue().Set(js_obj);
    return;
  }

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionSendResultResponse) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[1]).ToLocalChecked())
          ->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[2]).ToLocalChecked());

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_send_result_response(action_server, header, buffer),
      RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionTakeResultResponse) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  rcl_ret_t ret =
      rcl_action_take_result_response(action_client, header, buffer);
  int64_t sequence_number = header->sequence_number;
  free(header);

  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    v8::Local<v8::Integer> result =
        Nan::New<v8::Integer>(static_cast<int32_t>(sequence_number));
    info.GetReturnValue().Set(result);
    return;
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionAcceptNewGoal) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rcl_action_goal_info_t* buffer = reinterpret_cast<rcl_action_goal_info_t*>(
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked()));

  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(
          malloc(sizeof(rcl_action_goal_handle_t)));

  *goal_handle = *rcl_action_accept_new_goal(action_server, buffer);
  if (!goal_handle) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  auto js_obj = RclHandle::NewInstance(goal_handle, nullptr, [](void* ptr) {
    rcl_action_goal_handle_t* goal_handle =
        reinterpret_cast<rcl_action_goal_handle_t*>(ptr);
    rcl_ret_t ret = rcl_action_goal_handle_fini(goal_handle);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  });

  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(ActionUpdateGoalState) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* goal_handle_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(goal_handle_handle->ptr());
  rcl_action_goal_event_t event = static_cast<rcl_action_goal_event_t>(
      info[1]->IntegerValue(currentContent).FromJust());

  THROW_ERROR_IF_NOT_EQUAL(rcl_action_update_goal_state(goal_handle, event),
                           RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionPublishStatus) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
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

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionTakeStatus) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());

  rcl_ret_t ret = rcl_action_take_status(action_client, buffer);
  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::False());
    return;
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    info.GetReturnValue().Set(Nan::True());
    return;
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionGoalHandleIsActive) {
  RclHandle* goal_handle_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(goal_handle_handle->ptr());

  bool is_active = rcl_action_goal_handle_is_active(goal_handle);

  v8::Local<v8::Boolean> result = Nan::New<v8::Boolean>(is_active);
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionNotifyGoalDone) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(rcl_action_notify_goal_done(action_server),
                           RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionGoalHandleGetStatus) {
  RclHandle* goal_handle_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(goal_handle_handle->ptr());

  rcl_action_goal_state_t status;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_action_goal_handle_get_status(goal_handle, &status), RCL_RET_OK,
      rcl_get_error_string().str);

  v8::Local<v8::Integer> result =
      Nan::New<v8::Integer>(static_cast<int32_t>(status));
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionPublishFeedback) {
  rcl_action_server_t* action_server = reinterpret_cast<rcl_action_server_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());

  THROW_ERROR_IF_NOT_EQUAL(rcl_action_publish_feedback(action_server, buffer),
                           RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionTakeFeedback) {
  RclHandle* action_client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_client_t* action_client =
      reinterpret_cast<rcl_action_client_t*>(action_client_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());

  rcl_ret_t ret = rcl_action_take_feedback(action_client, buffer);
  if (ret != RCL_RET_OK && ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::False());
    return;
  }

  if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) {
    info.GetReturnValue().Set(Nan::True());
    return;
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ActionProcessCancelRequest) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_action_cancel_request_t* cancel_request =
      reinterpret_cast<rcl_action_cancel_request_t*>(buffer);
  void* response_buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[2]).ToLocalChecked());
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
    Nan::ThrowError(cancel_error.str);
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  *response = cancel_response_ptr->msg;
  auto js_obj =
      RclHandle::NewInstance(cancel_response_ptr, nullptr, [](void* ptr) {
        rcl_action_cancel_response_t* cancel_response_ptr =
            reinterpret_cast<rcl_action_cancel_response_t*>(ptr);
        rcl_ret_t ret = rcl_action_cancel_response_fini(cancel_response_ptr);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
      });
  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(ActionServerGoalExists) {
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rcl_action_goal_info_t* buffer = reinterpret_cast<rcl_action_goal_info_t*>(
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked()));

  bool exists = rcl_action_server_goal_exists(action_server, buffer);

  v8::Local<v8::Boolean> result = Nan::New<v8::Boolean>(exists);
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionExpireGoals) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* action_server_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  int64_t max_num_goals = info[1]->IntegerValue(currentContent).FromJust();
  rcl_action_goal_info_t* buffer = reinterpret_cast<rcl_action_goal_info_t*>(
      node::Buffer::Data(Nan::To<v8::Object>(info[2]).ToLocalChecked()));

  size_t num_expired;
  THROW_ERROR_IF_NOT_EQUAL(rcl_action_expire_goals(action_server, buffer,
                                                   max_num_goals, &num_expired),
                           RCL_RET_OK, rcl_get_error_string().str);

  v8::Local<v8::Integer> result =
      Nan::New<v8::Integer>(static_cast<int32_t>(num_expired));
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ActionGetClientNamesAndTypesByNode) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name =
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked());
  std::string node_namespace =
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked());

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_action_get_client_names_and_types_by_node(
                               node, &allocator, node_name.c_str(),
                               node_namespace.c_str(), &names_and_types),
                           "Failed to action client names and types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(ActionGetServerNamesAndTypesByNode) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name =
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked());
  std::string node_namespace =
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked());

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_action_get_server_names_and_types_by_node(
                               node, &allocator, node_name.c_str(),
                               node_namespace.c_str(), &names_and_types),
                           "Failed to action server names and types");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(ActionGetNamesAndTypes) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_action_get_names_and_types(node, &allocator, &names_and_types),
      "Failed to action server names and types");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  info.GetReturnValue().Set(result_list);
}

std::vector<BindingMethod> action_binding_methods = {
    {"actionCreateClient", ActionCreateClient},
    {"actionCreateServer", ActionCreateServer},
    {"actionServerIsAvailable", ActionServerIsAvailable},
    {"actionSendGoalRequest", ActionSendGoalRequest},
    {"actionTakeGoalRequest", ActionTakeGoalRequest},
    {"actionSendGoalResponse", ActionSendGoalResponse},
    {"actionTakeGoalResponse", ActionTakeGoalResponse},
    {"actionSendCancelRequest", ActionSendCancelRequest},
    {"actionTakeCancelRequest", ActionTakeCancelRequest},
    {"actionSendCancelResponse", ActionSendCancelResponse},
    {"actionTakeCancelResponse", ActionTakeCancelResponse},
    {"actionSendResultRequest", ActionSendResultRequest},
    {"actionTakeResultRequest", ActionTakeResultRequest},
    {"actionSendResultResponse", ActionSendResultResponse},
    {"actionTakeResultResponse", ActionTakeResultResponse},
    {"actionAcceptNewGoal", ActionAcceptNewGoal},
    {"actionUpdateGoalState", ActionUpdateGoalState},
    {"actionPublishStatus", ActionPublishStatus},
    {"actionTakeStatus", ActionTakeStatus},
    {"actionGoalHandleIsActive", ActionGoalHandleIsActive},
    {"actionNotifyGoalDone", ActionNotifyGoalDone},
    {"actionGoalHandleGetStatus", ActionGoalHandleGetStatus},
    {"actionPublishFeedback", ActionPublishFeedback},
    {"actionTakeFeedback", ActionTakeFeedback},
    {"actionProcessCancelRequest", ActionProcessCancelRequest},
    {"actionServerGoalExists", ActionServerGoalExists},
    {"actionExpireGoals", ActionExpireGoals},
    {"actionGetClientNamesAndTypesByNode", ActionGetClientNamesAndTypesByNode},
    {"actionGetServerNamesAndTypesByNode", ActionGetServerNamesAndTypesByNode},
    {"actionGetNamesAndTypes", ActionGetNamesAndTypes}};

}  // namespace rclnodejs
