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

#include <memory>
#include <mutex>
#include <unordered_map>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

struct TimerContext {
  Napi::ThreadSafeFunction on_reset_callback;
};

static std::unordered_map<rcl_timer_t*, std::shared_ptr<TimerContext>>
    g_timer_contexts;
static std::mutex g_timer_contexts_mutex;

void TimerOnResetCallbackTrampoline(const void* user_data,
                                    size_t number_of_events) {
  const rcl_timer_t* timer = static_cast<const rcl_timer_t*>(user_data);
  std::shared_ptr<TimerContext> context;

  {
    std::lock_guard<std::mutex> lock(g_timer_contexts_mutex);
    auto it = g_timer_contexts.find(const_cast<rcl_timer_t*>(timer));
    if (it != g_timer_contexts.end()) {
      context = it->second;
    }
  }

  if (context) {
    auto callback = [](Napi::Env env, Napi::Function js_callback,
                       size_t* events) {
      js_callback.Call({Napi::Number::New(env, *events)});
      delete events;
    };
    size_t* events_ptr = new size_t(number_of_events);
    context->on_reset_callback.BlockingCall(events_ptr, callback);
  }
}

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

#if ROS_VERSION > 2205
        // Clear the callback first to prevent any new callbacks from being
        // triggered
        rcl_ret_t callback_ret =
            rcl_timer_set_on_reset_callback(timer, nullptr, nullptr);
        THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, callback_ret,
                                           rcl_get_error_string().str);
#endif

        std::shared_ptr<TimerContext> context;
        {
          std::lock_guard<std::mutex> lock(g_timer_contexts_mutex);
          auto it = g_timer_contexts.find(timer);
          if (it != g_timer_contexts.end()) {
            context = it->second;
            g_timer_contexts.erase(it);
          }
        }

        if (context) {
          context->on_reset_callback.Release();
        }

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
Napi::Value GetTimerNextCallTime(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  int64_t next_call_time = 0;

  rcl_ret_t ret = rcl_timer_get_next_call_time(timer, &next_call_time);

  if (ret == RCL_RET_OK) {
    return Napi::BigInt::New(env, next_call_time);
  } else if (ret == RCL_RET_TIMER_CANCELED) {
    return env.Null();
  } else {
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
    return env.Undefined();  // Safeguard return, should not reach here
  }
}
#endif

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

Napi::Value SetTimerOnResetCallback(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  if (!info[1].IsFunction()) {
    Napi::TypeError::New(env, "Callback must be a function")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  Napi::Function callback = info[1].As<Napi::Function>();

  std::lock_guard<std::mutex> lock(g_timer_contexts_mutex);
  std::shared_ptr<TimerContext> context;
  auto it = g_timer_contexts.find(timer);
  if (it == g_timer_contexts.end()) {
    context = std::make_shared<TimerContext>();
    g_timer_contexts[timer] = context;
  } else {
    context = it->second;
    context->on_reset_callback.Release();
  }

  context->on_reset_callback = Napi::ThreadSafeFunction::New(
      env, callback, "TimerOnResetCallback", 0, 1);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_set_on_reset_callback(
                               timer, TimerOnResetCallbackTrampoline, timer),
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value ClearTimerOnResetCallback(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* timer_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  std::lock_guard<std::mutex> lock(g_timer_contexts_mutex);
  auto it = g_timer_contexts.find(timer);
  if (it != g_timer_contexts.end()) {
    it->second->on_reset_callback.Release();
    g_timer_contexts.erase(it);
  }

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_timer_set_on_reset_callback(timer, nullptr, nullptr),
      rcl_get_error_string().str);

  return env.Undefined();
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
  exports.Set("getTimerNextCallTime",
              Napi::Function::New(env, GetTimerNextCallTime));
  exports.Set("setTimerOnResetCallback",
              Napi::Function::New(env, SetTimerOnResetCallback));
  exports.Set("clearTimerOnResetCallback",
              Napi::Function::New(env, ClearTimerOnResetCallback));
  exports.Set("callTimerWithInfo", Napi::Function::New(env, CallTimerWithInfo));
#endif
  return exports;
}

}  // namespace rclnodejs
