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

#include "rcl_guard_condition_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value CreateGuardCondition(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_guard_condition_t* gc = reinterpret_cast<rcl_guard_condition_t*>(
      malloc(sizeof(rcl_guard_condition_t)));

  *gc = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t gc_options =
      rcl_guard_condition_get_default_options();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_guard_condition_init(gc, context, gc_options),
                           rcl_get_error_string().str);

  auto handle = RclHandle::NewInstance(env, gc, nullptr, [env](void* ptr) {
    rcl_guard_condition_t* gc = reinterpret_cast<rcl_guard_condition_t*>(ptr);
    rcl_ret_t ret = rcl_guard_condition_fini(gc);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                       rcl_get_error_string().str);
  });

  return handle;
}

Napi::Value TriggerGuardCondition(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* gc_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_guard_condition_t* gc =
      reinterpret_cast<rcl_guard_condition_t*>(gc_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_trigger_guard_condition(gc),
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Object InitGuardConditionBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createGuardCondition",
              Napi::Function::New(env, CreateGuardCondition));
  exports.Set("triggerGuardCondition",
              Napi::Function::New(env, TriggerGuardCondition));
  return exports;
}

}  // namespace rclnodejs
