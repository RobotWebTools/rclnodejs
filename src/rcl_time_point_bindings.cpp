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

#include "rcl_time_point_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <memory>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value CreateTimePoint(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  if (!info[0].IsBigInt()) {
    Napi::TypeError::New(env, "Timer period must be a BigInt")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  bool lossless;
  int64_t nanoseconds = info[0].As<Napi::BigInt>().Int64Value(&lossless);
  uint32_t clock_type = info[1].As<Napi::Number>().Uint32Value();
  rcl_time_point_t* time_point =
      reinterpret_cast<rcl_time_point_t*>(malloc(sizeof(rcl_time_point_t)));

  time_point->nanoseconds = nanoseconds;
  time_point->clock_type = static_cast<rcl_clock_type_t>(clock_type);

  auto js_obj = RclHandle::NewInstance(env, time_point, nullptr,
                                       [](void* ptr) { free(ptr); });

  return js_obj;
}

Napi::Value GetNanoseconds(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* time_point_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_time_point_t* time_point =
      reinterpret_cast<rcl_time_point_t*>(time_point_handle->ptr());

  return Napi::BigInt::New(env, time_point->nanoseconds);
}

Napi::Value CreateDuration(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  if (!info[0].IsBigInt()) {
    Napi::TypeError::New(env, "Timer period must be a BigInt")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  bool lossless;
  int64_t nanoseconds = info[0].As<Napi::BigInt>().Int64Value(&lossless);
  rcl_duration_t* duration =
      reinterpret_cast<rcl_duration_t*>(malloc(sizeof(rcl_duration_t)));
  duration->nanoseconds = nanoseconds;

  auto js_obj = RclHandle::NewInstance(env, duration, nullptr,
                                       [](void* ptr) { free(ptr); });

  return js_obj;
}

Napi::Value GetDurationNanoseconds(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* duration_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_duration_t* duration =
      reinterpret_cast<rcl_duration_t*>(duration_handle->ptr());

  return Napi::BigInt::New(env, duration->nanoseconds);
}

Napi::Value SetRosTimeOverrideIsEnabled(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* clock_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());
  bool enabled = info[1].As<Napi::Boolean>();

  if (enabled) {
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_enable_ros_time_override(clock),
                             rcl_get_error_string().str);
  } else {
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_disable_ros_time_override(clock),
                             rcl_get_error_string().str);
  }

  return env.Undefined();
}

Napi::Value SetRosTimeOverride(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* clock_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());
  RclHandle* time_point_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_time_point_t* time_point =
      reinterpret_cast<rcl_time_point_t*>(time_point_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_set_ros_time_override(clock, time_point->nanoseconds),
      rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value GetRosTimeOverrideIsEnabled(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* clock_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());

  bool is_enabled;
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_is_enabled_ros_time_override(clock, &is_enabled),
                           rcl_get_error_string().str);

  return Napi::Boolean::New(env, is_enabled);
}

Napi::Value CreateClock(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  auto clock_type =
      static_cast<rcl_clock_type_t>(info[0].As<Napi::Number>().Int32Value());
  rcl_clock_t* clock =
      reinterpret_cast<rcl_clock_t*>(malloc(sizeof(rcl_clock_t)));
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_clock_init(clock_type, clock, &allocator),
                           rcl_get_error_string().str);

  return RclHandle::NewInstance(env, clock, nullptr, [env](void* ptr) {
    rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(ptr);
    rcl_ret_t ret = rcl_clock_fini(clock);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                       rcl_get_error_string().str);
  });
}

Napi::Value ClockGetNow(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  rcl_time_point_t time_point;
  time_point.clock_type = clock->type;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_clock_get_now(clock, &time_point.nanoseconds),
                           rcl_get_error_string().str);

  return Napi::BigInt::New(env, time_point.nanoseconds);
}

struct JumpCallbackData {
  Napi::ThreadSafeFunction tsfn_pre;
  Napi::ThreadSafeFunction tsfn_post;
};

struct JumpCallbackContext {
  rcl_time_jump_t time_jump;
  bool before_jump;
};

void _rclnodejs_on_time_jump(const rcl_time_jump_t* time_jump, bool before_jump,
                             void* user_data) {
  JumpCallbackData* data = static_cast<JumpCallbackData*>(user_data);

  auto context = new JumpCallbackContext{*time_jump, before_jump};

  if (before_jump) {
    auto callback = [](Napi::Env env, Napi::Function js_callback,
                       JumpCallbackContext* context) {
      js_callback.Call({});
      delete context;
    };
    data->tsfn_pre.NonBlockingCall(context, callback);
  } else {
    auto callback = [](Napi::Env env, Napi::Function js_callback,
                       JumpCallbackContext* context) {
      Napi::Object jump_info = Napi::Object::New(env);
      jump_info.Set("clock_change",
                    static_cast<int32_t>(context->time_jump.clock_change));
      jump_info.Set("delta", Napi::BigInt::New(
                                 env, context->time_jump.delta.nanoseconds));
      js_callback.Call({jump_info});
      delete context;
    };
    data->tsfn_post.NonBlockingCall(context, callback);
  }
}

Napi::Value ClockAddJumpCallback(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* clock_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());

  Napi::Object callback_obj = info[1].As<Napi::Object>();
  Napi::Function pre_callback =
      callback_obj.Get("_pre_callback").As<Napi::Function>();
  Napi::Function post_callback =
      callback_obj.Get("_post_callback").As<Napi::Function>();

  bool on_clock_change = info[2].As<Napi::Boolean>();

  bool lossless;
  int64_t min_forward = info[3].As<Napi::BigInt>().Int64Value(&lossless);
  if (!lossless) {
    Napi::TypeError::New(
        env, "min_forward BigInt value cannot be represented as int64_t")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }
  int64_t min_backward = info[4].As<Napi::BigInt>().Int64Value(&lossless);
  if (!lossless) {
    Napi::TypeError::New(
        env, "min_backward BigInt value cannot be represented as int64_t")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  rcl_jump_threshold_t threshold;
  threshold.on_clock_change = on_clock_change;
  threshold.min_forward.nanoseconds = min_forward;
  threshold.min_backward.nanoseconds = min_backward;

  JumpCallbackData* data = new JumpCallbackData();
  data->tsfn_pre = Napi::ThreadSafeFunction::New(
      env, pre_callback, "ClockJumpPreCallback", 10, 1, [](Napi::Env) {});
  data->tsfn_post =
      Napi::ThreadSafeFunction::New(env, post_callback, "ClockJumpPostCallback",
                                    10, 1, [data](Napi::Env) { delete data; });

  Napi::Object handle_obj = Napi::Object::New(env);
  handle_obj.Set("_cpp_handle",
                 Napi::External<JumpCallbackData>::New(env, data));

  rcl_ret_t ret = rcl_clock_add_jump_callback(clock, threshold,
                                              _rclnodejs_on_time_jump, data);

  if (ret != RCL_RET_OK) {
    data->tsfn_pre.Release();
    data->tsfn_post.Release();
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  }

  callback_obj.Set("_cpp_handle", handle_obj.Get("_cpp_handle"));

  return env.Undefined();
}

Napi::Value ClockRemoveJumpCallback(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* clock_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());

  Napi::Object handle_obj = info[1].As<Napi::Object>();
  Napi::Value cpp_handle = handle_obj.Get("_cpp_handle");

  if (cpp_handle.IsUndefined() || !cpp_handle.IsExternal()) {
    Napi::Error::New(env,
                     "Callback object was not registered or already removed")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  JumpCallbackData* data =
      cpp_handle.As<Napi::External<JumpCallbackData>>().Data();

  rcl_ret_t ret =
      rcl_clock_remove_jump_callback(clock, _rclnodejs_on_time_jump, data);

  if (ret == RCL_RET_OK) {
    data->tsfn_pre.Release();
    data->tsfn_post.Release();
    handle_obj.Set("_cpp_handle", env.Undefined());
  } else {
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  }

  return env.Undefined();
}

Napi::Object InitTimePointBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createClock", Napi::Function::New(env, CreateClock));
  exports.Set("clockGetNow", Napi::Function::New(env, ClockGetNow));
  exports.Set("clockAddJumpCallback",
              Napi::Function::New(env, ClockAddJumpCallback));
  exports.Set("clockRemoveJumpCallback",
              Napi::Function::New(env, ClockRemoveJumpCallback));
  exports.Set("createTimePoint", Napi::Function::New(env, CreateTimePoint));
  exports.Set("getNanoseconds", Napi::Function::New(env, GetNanoseconds));
  exports.Set("createDuration", Napi::Function::New(env, CreateDuration));
  exports.Set("getDurationNanoseconds",
              Napi::Function::New(env, GetDurationNanoseconds));
  exports.Set("setRosTimeOverrideIsEnabled",
              Napi::Function::New(env, SetRosTimeOverrideIsEnabled));
  exports.Set("setRosTimeOverride",
              Napi::Function::New(env, SetRosTimeOverride));
  exports.Set("getRosTimeOverrideIsEnabled",
              Napi::Function::New(env, GetRosTimeOverrideIsEnabled));
  return exports;
}

}  // namespace rclnodejs
