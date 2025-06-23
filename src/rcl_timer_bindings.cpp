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

#include "rcl_timer_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value CreateTimer(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* clock_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());

  RclHandle* context_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  if (!info[2].IsBigInt()) {
    Napi::TypeError::New(env, "Timer period must be a BigInt")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  bool lossless;
  int64_t period_nsec = info[2].As<Napi::BigInt>().Int64Value(&lossless);
  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(malloc(sizeof(rcl_timer_t)));
  *timer = rcl_get_zero_initialized_timer();

#if ROS_VERSION > 2305  // After Iron.
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_timer_init2(timer, clock, context, period_nsec, nullptr,
                      rcl_get_default_allocator(), /*autostart=*/true),
      rcl_get_error_string().str);
#else
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_init(timer, clock, context, period_nsec,
                                          nullptr, rcl_get_default_allocator()),
                           rcl_get_error_string().str);
#endif

  auto js_obj =
      RclHandle::NewInstance(env, timer, clock_handle, [env](void* ptr) {
        rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(ptr);
        rcl_ret_t ret = rcl_timer_fini(timer);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                           rcl_get_error_string().str);
      });

  return js_obj;
}

Napi::Value IsTimerReady(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  bool is_ready = false;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_is_ready(timer, &is_ready),
                           rcl_get_error_string().str);

  return Napi::Boolean::New(env, is_ready);
}

Napi::Value CallTimer(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_call(timer),
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value CancelTimer(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_cancel(timer),
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value IsTimerCanceled(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  bool is_canceled = false;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_is_canceled(timer, &is_canceled),
                           rcl_get_error_string().str);

  return Napi::Boolean::New(env, is_canceled);
}

Napi::Value ResetTimer(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_reset(timer),
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value TimerGetTimeUntilNextCall(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  int64_t remaining_time = 0;

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_timer_get_time_until_next_call(timer, &remaining_time),
      rcl_get_error_string().str);

  return Napi::BigInt::New(env, remaining_time);
}

Napi::Value TimerGetTimeSinceLastCall(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  int64_t elapsed_time = 0;

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_timer_get_time_since_last_call(timer, &elapsed_time),
      rcl_get_error_string().str);

  return Napi::BigInt::New(env, elapsed_time);
}

Napi::Value ChangeTimerPeriod(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  if (!info[1].IsBigInt()) {
    Napi::TypeError::New(env, "Timer period must be a BigInt")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  bool lossless;
  int64_t period_nsec = info[1].As<Napi::BigInt>().Int64Value(&lossless);
  int64_t old_period;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_timer_exchange_period(timer, period_nsec, &old_period),
      rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value GetTimerPeriod(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  int64_t period_nsec = 0;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_get_period(timer, &period_nsec),
                           rcl_get_error_string().str);

  return Napi::BigInt::New(env, period_nsec);
}

#if ROS_VERSION > 2205  // 2205 == Humble
Napi::Value CallTimerWithInfo(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  rcl_timer_call_info_t call_info;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_call_with_info(timer, &call_info),
                           rcl_get_error_string().str);

  Napi::Object timer_info = Napi::Object::New(env);
  timer_info.Set("expectedCallTime",
                 Napi::BigInt::New(env, call_info.expected_call_time));
  timer_info.Set("actualCallTime",
                 Napi::BigInt::New(env, call_info.actual_call_time));
  return timer_info;
}
#endif

Napi::Object InitTimerBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createTimer", Napi::Function::New(env, CreateTimer));
  exports.Set("isTimerReady", Napi::Function::New(env, IsTimerReady));
  exports.Set("callTimer", Napi::Function::New(env, CallTimer));
  exports.Set("cancelTimer", Napi::Function::New(env, CancelTimer));
  exports.Set("isTimerCanceled", Napi::Function::New(env, IsTimerCanceled));
  exports.Set("resetTimer", Napi::Function::New(env, ResetTimer));
  exports.Set("timerGetTimeSinceLastCall",
              Napi::Function::New(env, TimerGetTimeSinceLastCall));
  exports.Set("timerGetTimeUntilNextCall",
              Napi::Function::New(env, TimerGetTimeUntilNextCall));
  exports.Set("changeTimerPeriod", Napi::Function::New(env, ChangeTimerPeriod));
  exports.Set("getTimerPeriod", Napi::Function::New(env, GetTimerPeriod));
#if ROS_VERSION > 2205  // 2205 == Humble
  exports.Set("callTimerWithInfo", Napi::Function::New(env, CallTimerWithInfo));
#endif
  return exports;
}

}  // namespace rclnodejs
