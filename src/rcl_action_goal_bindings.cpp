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

#include "rcl_action_goal_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>

#include "handle_manager.h"
#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value ActionAcceptNewGoal(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* action_server_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_server_t* action_server =
      reinterpret_cast<rcl_action_server_t*>(action_server_handle->ptr());
  rcl_action_goal_info_t* buffer = reinterpret_cast<rcl_action_goal_info_t*>(
      info[1].As<Napi::Buffer<char>>().Data());

  rcl_action_goal_handle_t* new_goal =
      rcl_action_accept_new_goal(action_server, buffer);
  if (!new_goal) {
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcl_reset_error();
    return env.Undefined();
  }

  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(
          malloc(sizeof(rcl_action_goal_handle_t)));
  *goal_handle = *new_goal;
  auto js_obj =
      RclHandle::NewInstance(env, goal_handle, nullptr, [env](void* ptr) {
        rcl_action_goal_handle_t* goal_handle =
            reinterpret_cast<rcl_action_goal_handle_t*>(ptr);
        rcl_ret_t ret = rcl_action_goal_handle_fini(goal_handle);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                           rcl_get_error_string().str);
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

Napi::Value ActionGoalHandleIsActive(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* goal_handle_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_action_goal_handle_t* goal_handle =
      reinterpret_cast<rcl_action_goal_handle_t*>(goal_handle_handle->ptr());

  bool is_active = rcl_action_goal_handle_is_active(goal_handle);

  return Napi::Boolean::New(env, is_active);
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

Napi::Object InitActionGoalBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("actionAcceptNewGoal",
              Napi::Function::New(env, ActionAcceptNewGoal));
  exports.Set("actionUpdateGoalState",
              Napi::Function::New(env, ActionUpdateGoalState));
  exports.Set("actionGoalHandleIsActive",
              Napi::Function::New(env, ActionGoalHandleIsActive));
  exports.Set("actionGoalHandleGetStatus",
              Napi::Function::New(env, ActionGoalHandleGetStatus));
  return exports;
}

}  // namespace rclnodejs
