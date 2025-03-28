// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

#include "rcl_bindings.hpp"

#include <napi.h>
#include <node.h>
#include <rcl/arguments.h>
#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/graph.h>
#include <rcl/logging.h>
#include <rcl/node.h>
#include <rcl/rcl.h>
#include <rcl/validate_topic_name.h>
#include <rcl_action/rcl_action.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcl_yaml_param_parser/types.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>

#if ROS_VERSION >= 2006
#include <rosidl_runtime_c/string_functions.h>
#else
#include <rosidl_generator_c/string_functions.h>
#endif

#if ROS_VERSION > 2205
#include <rcl/service_introspection.h>
#endif

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#if NODE_MAJOR_VERSION > 12
#include <utility>
#endif

#include "handle_manager.hpp"
#include "macros.hpp"
#include "rcl_handle.hpp"
#include "rcl_utilities.hpp"

namespace rclnodejs {

static Napi::Object wrapParameters(
    Napi::Env env, rcl_params_t* params);  // NOLINT(whitespace/line_length)

Napi::Value InitRclnodejs(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_init_options_init(&init_options, allocator),
                           rcl_get_error_string().str);

  // preprocess Context
  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  // preprocess argc & argv
  Napi::Array jsArgv = info[1].As<Napi::Array>();
  int argc = jsArgv.Length();
  char** argv = nullptr;

  if (argc > 0) {
    argv = reinterpret_cast<char**>(malloc(argc * sizeof(char*)));
    for (int i = 0; i < argc; i++) {
      std::string arg = jsArgv.Get(i).As<Napi::String>().Utf8Value();
      int len = arg.length() + 1;
      argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char*)));
      snprintf(argv[i], len, "%s", arg.c_str());
    }
  }

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_init(argc, argc > 0 ? argv : nullptr, &init_options, context),
      rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_logging_configure(&context->global_arguments, &allocator),
      rcl_get_error_string().str);

  for (int i = 0; i < argc; i++) {
    free(argv[i]);
  }
  free(argv);

  return env.Undefined();
}

Napi::Value GetParameterOverrides(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_arguments_t* parsed_args = &(context->global_arguments);
  rcl_params_t* params = NULL;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_arguments_get_param_overrides(parsed_args, &params),
      rcl_get_error_string().str);

  if (params == NULL) {
    return env.Undefined();
  }

  Napi::Object result = wrapParameters(env, params);

  rcl_yaml_node_struct_fini(params);
  return result;
}

static const int PARAMETER_NOT_SET = 0;
static const int PARAMETER_BOOL = 1;
static const int PARAMETER_INTEGER = 2;
static const int PARAMETER_DOUBLE = 3;
static const int PARAMETER_STRING = 4;
static const int PARAMETER_BYTE_ARRAY = 5;
static const int PARAMETER_BOOL_ARRAY = 6;
static const int PARAMETER_INTEGER_ARRAY = 7;
static const int PARAMETER_DOUBLE_ARRAY = 8;
static const int PARAMETER_STRING_ARRAY = 9;

/*
Convert parsed ros arguments to parameters.

type Parameter = {
  name: string,
  type: number,
  value: object
}

type Node = {
  name: string,
  parameters: array<Parameter>
}

parameters = array<Node>;
*/
static Napi::Object wrapParameters(Napi::Env env, rcl_params_t* parsed_args) {
  Napi::Array nodes = Napi::Array::New(env);

  // iterate over nodes
  for (size_t i = 0; i < parsed_args->num_nodes; i++) {
    Napi::Object node = Napi::Object::New(env);
    node.Set("name", Napi::String::New(env, parsed_args->node_names[i]));

    rcl_node_params_t node_parameters = parsed_args->params[i];

    // iterate over node.parameters
    Napi::Array parameters = Napi::Array::New(env);
    for (size_t j = 0; j < node_parameters.num_params; j++) {
      Napi::Object parameter = Napi::Object::New(env);

      parameter.Set(
          "name",
          Napi::String::New(env, parsed_args->params[i].parameter_names[j]));

      int param_type = PARAMETER_NOT_SET;

      // for each value, find type & actual value
      rcl_variant_t value = node_parameters.parameter_values[j];
      if (value.bool_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BOOL;
        parameter.Set("value", Napi::Boolean::New(env, *value.bool_value));
      } else if (value.integer_value != NULL) {  // NOLINT()
        param_type = PARAMETER_INTEGER;
        parameter.Set("value", Napi::Number::New(env, *value.integer_value));
      } else if (value.double_value != NULL) {  // NOLINT()
        param_type = PARAMETER_DOUBLE;
        parameter.Set("value", Napi::Number::New(env, *value.double_value));
      } else if (value.string_value != NULL) {  // NOLINT()
        param_type = PARAMETER_STRING;
        parameter.Set("value", Napi::String::New(env, value.string_value));
      } else if (value.bool_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BOOL_ARRAY;
        Napi::Array bool_array = Napi::Array::New(env);

        for (size_t k = 0; k < value.bool_array_value->size; k++) {
          bool_array.Set(
              k, Napi::Boolean::New(env, value.bool_array_value->values[k]));
        }
        parameter.Set("value", bool_array);
      } else if (value.string_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_STRING_ARRAY;
        Napi::Array string_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.string_array_value->size; k++) {
          string_array.Set(
              k, Napi::String::New(env, value.string_array_value->data[k]));
        }
        parameter.Set("value", string_array);
      } else if (value.byte_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BYTE_ARRAY;
        Napi::Array byte_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.byte_array_value->size; k++) {
          byte_array.Set(
              k, Napi::Number::New(env, value.byte_array_value->values[k]));
        }
        parameter.Set("value", byte_array);
      } else if (value.integer_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_INTEGER_ARRAY;
        Napi::Array int_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.integer_array_value->size; k++) {
          int_array.Set(
              k, Napi::Number::New(env, value.integer_array_value->values[k]));
        }
        parameter.Set("value", int_array);
      } else if (value.double_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_DOUBLE_ARRAY;
        Napi::Array dbl_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.double_array_value->size; k++) {
          dbl_array.Set(
              k, Napi::Number::New(env, value.double_array_value->values[k]));
        }
        parameter.Set("value", dbl_array);
      }

      parameter.Set("type", Napi::Number::New(env, param_type));
      parameters.Set(j, parameter);
    }

    node.Set("parameters", parameters);
    nodes.Set(i, node);
  }

  return nodes;
}

Napi::Value CreateNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string node_name = info[0].As<Napi::String>().Utf8Value();
  std::string name_space = info[1].As<Napi::String>().Utf8Value();
  RclHandle* context_handle = RclHandle::Unwrap(info[2].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(malloc(sizeof(rcl_node_t)));

  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_node_init(node, node_name.c_str(),
                                         name_space.c_str(), context, &options),
                           rcl_get_error_string().str);

  auto handle = RclHandle::NewInstance(env, node, nullptr, [](void* ptr) {
    rcl_node_t* node = reinterpret_cast<rcl_node_t*>(ptr);
    rcl_ret_t ret = rcl_node_fini(node);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  });

  return handle;
}

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

  auto handle = RclHandle::NewInstance(env, gc, nullptr, [](void* ptr) {
    rcl_guard_condition_t* gc = reinterpret_cast<rcl_guard_condition_t*>(ptr);
    rcl_ret_t ret = rcl_guard_condition_fini(gc);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
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

  auto js_obj = RclHandle::NewInstance(env, timer, clock_handle, [](void* ptr) {
    rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(ptr);
    rcl_ret_t ret = rcl_timer_fini(timer);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
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

  return RclHandle::NewInstance(env, clock, nullptr, [](void* ptr) {
    rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(ptr);
    rcl_ret_t ret = rcl_clock_fini(clock);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
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

Napi::Value RclTake(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());
  void* msg_taken = info[1].As<Napi::Buffer<char>>().Data();
  rcl_ret_t ret = rcl_take(subscription, msg_taken, nullptr, nullptr);

  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    rcl_reset_error();
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    return Napi::Boolean::New(env, false);
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    return Napi::Boolean::New(env, true);
  }

  return env.Undefined();
}

Napi::Value CreateSubscription(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  std::string package_name = info[1].As<Napi::String>().Utf8Value();
  std::string message_sub_folder = info[2].As<Napi::String>().Utf8Value();
  std::string message_name = info[3].As<Napi::String>().Utf8Value();
  std::string topic = info[4].As<Napi::String>().Utf8Value();
  Napi::Object options = info[5].As<Napi::Object>();

  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(malloc(sizeof(rcl_subscription_t)));
  *subscription = rcl_get_zero_initialized_subscription();

  rcl_subscription_options_t subscription_ops =
      rcl_subscription_get_default_options();

  Napi::Value qos = options.Get("qos");
  auto qos_profile = GetQoSProfile(qos);
  if (qos_profile) {
    subscription_ops.qos = *qos_profile;
  }

#if ROS_VERSION >= 2205  // 2205 => Humble+
  if (options.Has("contentFilter")) {
    // configure content-filter
    Napi::Value contentFilterVal = options.Get("contentFilter");

    if (!contentFilterVal.IsUndefined()) {
      Napi::Object contentFilter = contentFilterVal.As<Napi::Object>();

      // expression property is required
      std::string expression =
          contentFilter.Get("expression").As<Napi::String>().Utf8Value();

      // parameters property (string[]) is optional
      int argc = 0;
      char** argv = nullptr;

      if (contentFilter.Has("parameters")) {
        Napi::Array jsArgv = contentFilter.Get("parameters").As<Napi::Array>();
        argc = jsArgv.Length();
        if (argc > 0) {
          argv = reinterpret_cast<char**>(malloc(argc * sizeof(char*)));
          for (int i = 0; i < argc; i++) {
            std::string arg = jsArgv.Get(i).As<Napi::String>().Utf8Value();
            int len = arg.length() + 1;
            argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char*)));
            snprintf(argv[i], len, "%s", arg.c_str());
          }
        }
      }

      rcl_ret_t ret = rcl_subscription_options_set_content_filter_options(
          expression.c_str(), argc, (const char**)argv, &subscription_ops);

      if (ret != RCL_RET_OK) {
        rcl_reset_error();
        Napi::Error::New(env, rcl_get_error_string().str)
            .ThrowAsJavaScriptException();
      }

      if (argc) {
        for (int i = 0; i < argc; i++) {
          free(argv[i]);
        }
        free(argv);
      }
    }
  }
#endif

  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);

  if (ts) {
    THROW_ERROR_IF_NOT_EQUAL(
        RCL_RET_OK,
        rcl_subscription_init(subscription, node, ts, topic.c_str(),
                              &subscription_ops),
        rcl_get_error_string().str);

    auto js_obj = RclHandle::NewInstance(
        env, subscription, node_handle, [node](void* ptr) {
          rcl_subscription_t* subscription =
              reinterpret_cast<rcl_subscription_t*>(ptr);
          rcl_ret_t ret = rcl_subscription_fini(subscription, node);
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

Napi::Value HasContentFilter(const Napi::CallbackInfo& info) {
#if ROS_VERSION < 2205  // 2205 => Humble+
  return Napi::Boolean::New(info.Env(), false);
#else
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  bool is_valid = rcl_subscription_is_cft_enabled(subscription);
  return Napi::Boolean::New(env, is_valid);
#endif
}

Napi::Value SetContentFilter(const Napi::CallbackInfo& info) {
#if ROS_VERSION < 2205  // 2205 => Humble+
  return Napi::Boolean::New(info.Env(), false);
#else
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  Napi::Object contentFilter = info[1].As<Napi::Object>();

  std::string expression =
      contentFilter.Get("expression").As<Napi::String>().Utf8Value();

  // parameters property (string[]) is optional
  int argc = 0;
  char** argv = nullptr;

  if (contentFilter.Has("parameters")) {
    Napi::Array jsArgv = contentFilter.Get("parameters").As<Napi::Array>();
    argc = jsArgv.Length();
    if (argc > 0) {
      argv = reinterpret_cast<char**>(malloc(argc * sizeof(char*)));
      for (int i = 0; i < argc; i++) {
        std::string arg = jsArgv.Get(i).As<Napi::String>().Utf8Value();
        int len = arg.length() + 1;
        argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char*)));
        snprintf(argv[i], len, "%s", arg.c_str());
      }
    }
  }

  // create ctf options
  rcl_subscription_content_filter_options_t options =
      rcl_get_zero_initialized_subscription_content_filter_options();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_subscription_content_filter_options_set(
          subscription, expression.c_str(), argc, (const char**)argv, &options),
      rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_subscription_set_content_filter(subscription, &options),
      rcl_get_error_string().str);

  if (argc) {
    for (int i = 0; i < argc; i++) {
      free(argv[i]);
    }
    free(argv);
  }

  return Napi::Boolean::New(env, true);
#endif
}

Napi::Value ClearContentFilter(const Napi::CallbackInfo& info) {
#if ROS_VERSION < 2205  // 2205 => Humble+
  return Napi::Boolean::New(info.Env(), false);
#else
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  // create ctf options
  rcl_subscription_content_filter_options_t options =
      rcl_get_zero_initialized_subscription_content_filter_options();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_subscription_content_filter_options_init(
          subscription, "", 0, (const char**)nullptr, &options),
      rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_subscription_set_content_filter(subscription, &options),
      rcl_get_error_string().str);

  return Napi::Boolean::New(env, true);
#endif
}

Napi::Value CreatePublisher(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  std::string package_name = info[1].As<Napi::String>().Utf8Value();
  std::string message_sub_folder = info[2].As<Napi::String>().Utf8Value();
  std::string message_name = info[3].As<Napi::String>().Utf8Value();
  std::string topic = info[4].As<Napi::String>().Utf8Value();

  rcl_publisher_t* publisher =
      reinterpret_cast<rcl_publisher_t*>(malloc(sizeof(rcl_publisher_t)));
  *publisher = rcl_get_zero_initialized_publisher();

  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);

  if (ts) {
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    auto qos_profile = GetQoSProfile(info[5]);

    if (qos_profile) {
      publisher_ops.qos = *qos_profile;
    }

    THROW_ERROR_IF_NOT_EQUAL(
        rcl_publisher_init(publisher, node, ts, topic.c_str(), &publisher_ops),
        RCL_RET_OK, rcl_get_error_string().str);

    auto js_obj =
        RclHandle::NewInstance(env, publisher, node_handle, [node](void* ptr) {
          rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(ptr);
          rcl_ret_t ret = rcl_publisher_fini(publisher, node);
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

Napi::Value Publish(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  void* buffer = info[1].As<Napi::Buffer<char>>().Data();
  THROW_ERROR_IF_NOT_EQUAL(rcl_publish(publisher, buffer, nullptr), RCL_RET_OK,
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value GetPublisherTopic(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* topic = rcl_publisher_get_topic_name(publisher);
  return Napi::String::New(env, topic);
}

Napi::Value GetSubscriptionTopic(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_subscription_t* subscription = reinterpret_cast<rcl_subscription_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* topic = rcl_subscription_get_topic_name(subscription);
  return Napi::String::New(env, topic);
}

Napi::Value CreateClient(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  std::string service_name = info[1].As<Napi::String>().Utf8Value();
  std::string interface_name = info[2].As<Napi::String>().Utf8Value();
  std::string package_name = info[3].As<Napi::String>().Utf8Value();

  const rosidl_service_type_support_t* ts =
      GetServiceTypeSupport(package_name, interface_name);

  if (ts) {
    rcl_client_t* client =
        reinterpret_cast<rcl_client_t*>(malloc(sizeof(rcl_client_t)));
    *client = rcl_get_zero_initialized_client();
    rcl_client_options_t client_ops = rcl_client_get_default_options();
    auto qos_profile = GetQoSProfile(info[4]);

    if (qos_profile) {
      client_ops.qos = *qos_profile;
    }

    THROW_ERROR_IF_NOT_EQUAL(
        rcl_client_init(client, node, ts, service_name.c_str(), &client_ops),
        RCL_RET_OK, rcl_get_error_string().str);

    auto js_obj =
        RclHandle::NewInstance(env, client, node_handle, [node](void* ptr) {
          rcl_client_t* client = reinterpret_cast<rcl_client_t*>(ptr);
          rcl_ret_t ret = rcl_client_fini(client, node);
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

Napi::Value SendRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  char* buffer = info[1].As<Napi::Buffer<char>>().Data();
  int64_t sequence_number;
  THROW_ERROR_IF_NOT_EQUAL(rcl_send_request(client, buffer, &sequence_number),
                           RCL_RET_OK, rcl_get_error_string().str);

  return Napi::Number::New(env, static_cast<uint32_t>(sequence_number));
}

Napi::Value RclTakeResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  rmw_service_info_t header;

  void* taken_response = info[1].As<Napi::Buffer<char>>().Data();
  rcl_ret_t ret = rcl_take_response_with_info(client, &header, taken_response);
  int64_t sequence_number = header.request_id.sequence_number;

  if (ret == RCL_RET_OK) {
    return Napi::Number::New(env, static_cast<uint32_t>(sequence_number));
  }

  rcl_reset_error();
  return env.Undefined();
}

Napi::Value CreateService(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  std::string service_name = info[1].As<Napi::String>().Utf8Value();
  std::string interface_name = info[2].As<Napi::String>().Utf8Value();
  std::string package_name = info[3].As<Napi::String>().Utf8Value();

  const rosidl_service_type_support_t* ts =
      GetServiceTypeSupport(package_name, interface_name);

  if (ts) {
    rcl_service_t* service =
        reinterpret_cast<rcl_service_t*>(malloc(sizeof(rcl_service_t)));
    *service = rcl_get_zero_initialized_service();
    rcl_service_options_t service_ops = rcl_service_get_default_options();
    auto qos_profile = GetQoSProfile(info[4]);

    if (qos_profile) {
      service_ops.qos = *qos_profile;
    }

    THROW_ERROR_IF_NOT_EQUAL(
        rcl_service_init(service, node, ts, service_name.c_str(), &service_ops),
        RCL_RET_OK, rcl_get_error_string().str);
    auto js_obj =
        RclHandle::NewInstance(env, service, node_handle, [node](void* ptr) {
          rcl_service_t* service = reinterpret_cast<rcl_service_t*>(ptr);
          rcl_ret_t ret = rcl_service_fini(service, node);
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

Napi::Value RclTakeRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request = info[2].As<Napi::Buffer<char>>().Data();
  rcl_ret_t ret = rcl_take_request(service, header, taken_request);
  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    auto js_obj = RclHandle::NewInstance(env, header, nullptr,
                                         [](void* ptr) { free(ptr); });
    return js_obj;
  }

  return env.Undefined();
}

Napi::Value SendResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap(info[2].As<Napi::Object>())->ptr());

  THROW_ERROR_IF_NOT_EQUAL(rcl_send_response(service, header, buffer),
                           RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

#if ROS_VERSION > 2205  // 2205 == Humble
Napi::Value ConfigureServiceIntrospection(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      RclHandle::Unwrap(info[2].As<Napi::Object>())->ptr());

  std::string interface_name = info[3].As<Napi::String>().Utf8Value();
  std::string package_name = info[4].As<Napi::String>().Utf8Value();
  const rosidl_service_type_support_t* ts =
      GetServiceTypeSupport(package_name, interface_name);

  if (ts) {
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    auto qos_profile = GetQoSProfile(info[5]);
    if (qos_profile) {
      publisher_ops.qos = *qos_profile;
    }

    rcl_service_introspection_state_t state =
        static_cast<rcl_service_introspection_state_t>(
            info[6].As<Napi::Number>().Uint32Value());

    bool configureForService = info[7].As<Napi::Boolean>();

    if (configureForService) {
      RclHandle* service_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
      rcl_service_t* service =
          reinterpret_cast<rcl_service_t*>(service_handle->ptr());

      THROW_ERROR_IF_NOT_EQUAL(
          rcl_service_configure_service_introspection(service, node, clock, ts,
                                                      publisher_ops, state),
          RCL_RET_OK, rcl_get_error_string().str);

    } else {
      RclHandle* client_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
      rcl_client_t* client =
          reinterpret_cast<rcl_client_t*>(client_handle->ptr());

      THROW_ERROR_IF_NOT_EQUAL(
          rcl_client_configure_service_introspection(client, node, clock, ts,
                                                     publisher_ops, state),
          RCL_RET_OK, rcl_get_error_string().str);
    }

  } else {
    Napi::Error::New(env, GetErrorMessageAndClear())
        .ThrowAsJavaScriptException();
  }

  return env.Undefined();
}
#endif

Napi::Value ValidateFullTopicName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  int validation_result;
  size_t invalid_index;
  std::string topic_name = info[0].As<Napi::String>().Utf8Value();
  rmw_ret_t ret = rmw_validate_full_topic_name(
      topic_name.c_str(), &validation_result, &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Napi::Error::New(env, rmw_get_error_string().str)
          .ThrowAsJavaScriptException();
    }
    rmw_reset_error();
    return env.Undefined();
  }

  if (validation_result == RMW_NAMESPACE_VALID) {
    return env.Null();
  }
  const char* validation_message =
      rmw_full_topic_name_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  Napi::Array result_list = Napi::Array::New(env, 2);
  result_list.Set(static_cast<uint32_t>(0),
                  Napi::String::New(env, std::string(validation_message)));
  result_list.Set(static_cast<uint32_t>(1),
                  Napi::Number::New(env, static_cast<int32_t>(invalid_index)));

  return result_list;
}

Napi::Value ValidateNodeName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  int validation_result;
  size_t invalid_index;
  std::string node_name = info[0].As<Napi::String>().Utf8Value();
  rmw_ret_t ret = rmw_validate_node_name(node_name.c_str(), &validation_result,
                                         &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Napi::Error::New(env, rmw_get_error_string().str)
          .ThrowAsJavaScriptException();
    }
    rmw_reset_error();
    return env.Undefined();
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    return env.Null();
  }
  const char* validation_message =
      rmw_node_name_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  Napi::Array result_list = Napi::Array::New(env, 2);
  result_list.Set(static_cast<uint32_t>(0),
                  Napi::String::New(env, std::string(validation_message)));
  result_list.Set(static_cast<uint32_t>(1),
                  Napi::Number::New(env, static_cast<int32_t>(invalid_index)));

  return result_list;
}

Napi::Value ValidateTopicName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  int validation_result;
  size_t invalid_index;
  std::string topic_name = info[0].As<Napi::String>().Utf8Value();
  rmw_ret_t ret = rcl_validate_topic_name(topic_name.c_str(),
                                          &validation_result, &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Napi::Error::New(env, rmw_get_error_string().str)
          .ThrowAsJavaScriptException();
    }
    rmw_reset_error();
    return env.Undefined();
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    return env.Null();
  }
  const char* validation_message =
      rcl_topic_name_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  Napi::Array result_list = Napi::Array::New(env, 2);
  result_list.Set(static_cast<uint32_t>(0),
                  Napi::String::New(env, std::string(validation_message)));
  result_list.Set(static_cast<uint32_t>(1),
                  Napi::Number::New(env, static_cast<int32_t>(invalid_index)));

  return result_list;
}

Napi::Value ValidateNamespace(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  int validation_result;
  size_t invalid_index;
  std::string namespace_name = info[0].As<Napi::String>().Utf8Value();
  rmw_ret_t ret = rmw_validate_namespace(namespace_name.c_str(),
                                         &validation_result, &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Napi::Error::New(env, rmw_get_error_string().str)
          .ThrowAsJavaScriptException();
    }
    rmw_reset_error();
    return env.Undefined();
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    return env.Null();
  }
  const char* validation_message =
      rmw_namespace_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  Napi::Array result_list = Napi::Array::New(env, 2);
  result_list.Set(static_cast<uint32_t>(0),
                  Napi::String::New(env, std::string(validation_message)));
  result_list.Set(static_cast<uint32_t>(1),
                  Napi::Number::New(env, static_cast<int32_t>(invalid_index)));

  return result_list;
}

Napi::Value ExpandTopicName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string topic_name = info[0].As<Napi::String>().Utf8Value();
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();

  char* expanded_topic = nullptr;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_allocator_t rcutils_allocator = rcutils_get_default_allocator();
  rcutils_string_map_t substitutions_map =
      rcutils_get_zero_initialized_string_map();

  rcutils_ret_t rcutils_ret =
      rcutils_string_map_init(&substitutions_map, 0, rcutils_allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    if (rcutils_ret == RCUTILS_RET_BAD_ALLOC) {
      rcl_reset_error();
      Napi::Error::New(env, rcutils_get_error_string().str)
          .ThrowAsJavaScriptException();
    }
    rcutils_reset_error();
    return env.Undefined();
  }
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (ret != RCL_RET_OK) {
    rcl_reset_error();

    if (ret == RCL_RET_BAD_ALLOC) {
      Napi::Error::New(env, rcl_get_error_string().str)
          .ThrowAsJavaScriptException();
    }

    rcutils_ret = rcutils_string_map_fini(&substitutions_map);
    if (rcutils_ret != RCUTILS_RET_OK) {
      rcutils_reset_error();
      Napi::Error::New(env, rcutils_get_error_string().str)
          .ThrowAsJavaScriptException();
    }
    return env.Undefined();
  }

  ret = rcl_expand_topic_name(topic_name.c_str(), node_name.c_str(),
                              node_namespace.c_str(), &substitutions_map,
                              allocator, &expanded_topic);

  rcutils_ret = rcutils_string_map_fini(&substitutions_map);
  if (rcutils_ret != RCUTILS_RET_OK) {
    Napi::Error::New(env, rcutils_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcutils_reset_error();
    allocator.deallocate(expanded_topic, allocator.state);
    return env.Undefined();
  }
  if (ret != RCL_RET_OK) {
    rcl_reset_error();
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  if (!expanded_topic) {
    return env.Undefined();
  }

  rcl_allocator_t topic_allocator = rcl_get_default_allocator();
  std::string topic(expanded_topic);
  allocator.deallocate(expanded_topic, topic_allocator.state);
  return Napi::String::New(env, topic);
}

Napi::Value GetNodeName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* node_name = rcl_node_get_name(node);
  if (!node_name) {
    return env.Undefined();
  } else {
    return Napi::String::New(env, node_name);
  }
}

Napi::Value GetNamespace(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* node_namespace = rcl_node_get_namespace(node);
  if (!node_namespace) {
    return env.Undefined();
  } else {
    return Napi::String::New(env, node_namespace);
  }
}

const rmw_qos_profile_t* GetQoSProfileFromString(const std::string& profile) {
  const rmw_qos_profile_t* qos_profile = nullptr;
  if (profile == "qos_profile_sensor_data") {
    qos_profile = &rmw_qos_profile_sensor_data;
  } else if (profile == "qos_profile_system_default") {
    qos_profile = &rmw_qos_profile_system_default;
  } else if (profile == "qos_profile_services_default") {
    qos_profile = &rmw_qos_profile_services_default;
  } else if (profile == "qos_profile_parameters") {
    qos_profile = &rmw_qos_profile_parameters;
  } else if (profile == "qos_profile_parameter_events") {
    qos_profile = &rmw_qos_profile_parameter_events;
  } else if (profile == "qos_profile_action_status_default") {
    qos_profile = &rcl_action_qos_profile_status_default;
  } else {
    return &rmw_qos_profile_default;
  }

  return qos_profile;
}

std::unique_ptr<rmw_qos_profile_t> GetQosProfileFromObject(
    Napi::Object object) {
  std::unique_ptr<rmw_qos_profile_t> qos_profile =
      std::make_unique<rmw_qos_profile_t>();

  auto history = object.Get("history");
  auto depth = object.Get("depth");
  auto reliability = object.Get("reliability");
  auto durability = object.Get("durability");
  auto avoid_ros_namespace_conventions =
      object.Get("avoidRosNameSpaceConventions");

  qos_profile->history = static_cast<rmw_qos_history_policy_t>(
      history.As<Napi::Number>().Uint32Value());
  qos_profile->depth = depth.As<Napi::Number>().Uint32Value();
  qos_profile->reliability = static_cast<rmw_qos_reliability_policy_t>(
      reliability.As<Napi::Number>().Uint32Value());
  qos_profile->durability = static_cast<rmw_qos_durability_policy_t>(
      durability.As<Napi::Number>().Uint32Value());
  qos_profile->avoid_ros_namespace_conventions =
      avoid_ros_namespace_conventions.As<Napi::Boolean>();

  return qos_profile;
}

std::unique_ptr<rmw_qos_profile_t> GetQoSProfile(Napi::Value qos) {
  std::unique_ptr<rmw_qos_profile_t> qos_profile =
      std::make_unique<rmw_qos_profile_t>();

  if (qos.IsString()) {
    *qos_profile = *GetQoSProfileFromString(qos.As<Napi::String>().Utf8Value());
  } else if (qos.IsObject()) {
    qos_profile = GetQosProfileFromObject(qos.As<Napi::Object>());
  } else {
    return qos_profile;
  }
  return qos_profile;
}

int DestroyContext(Napi::Env env, rcl_context_t* context) {
  rcl_ret_t ret = RCL_RET_OK;
  if (context->impl) {
    if (rcl_context_is_valid(context)) {
      if (RCL_RET_OK != rcl_shutdown(context)) {
        Napi::Error::New(env, rcl_get_error_string().str)
            .ThrowAsJavaScriptException();
      }
      ret = rcl_context_fini(context);
    }
  }
  return ret;
}

Napi::Value Shutdown(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  THROW_ERROR_IF_NOT_EQUAL(rcl_shutdown(context), RCL_RET_OK,
                           rcl_get_error_string().str);
  THROW_ERROR_IF_NOT_EQUAL(rcl_logging_fini(), RCL_RET_OK,
                           rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value InitString(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  void* buffer = info[0].As<Napi::Buffer<char>>().Data();
#if ROS_VERSION >= 2006
  rosidl_runtime_c__String* ptr =
      reinterpret_cast<rosidl_runtime_c__String*>(buffer);

  rosidl_runtime_c__String__init(ptr);
#else
  rosidl_generator_c__String* ptr =
      reinterpret_cast<rosidl_generator_c__String*>(buffer);

  rosidl_generator_c__String__init(ptr);
#endif
  return env.Undefined();
}

inline char* GetBufAddr(Napi::Value buf) {
  return buf.As<Napi::Buffer<char>>().Data();
}

Napi::Value FreeMemeoryAtOffset(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string addr_str = info[0].As<Napi::String>().Utf8Value();
  int64_t result = std::stoull(addr_str, 0, 16);
  char* addr = reinterpret_cast<char*>(result);
  int64_t offset =
      info[1].IsNumber() ? info[1].As<Napi::Number>().Int64Value() : 0;
  auto ptr = addr + offset;

  char* val = *reinterpret_cast<char**>(ptr);
  free(val);
  return env.Undefined();
}

Napi::Value CreateArrayBufferFromAddress(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string addr_str = info[0].As<Napi::String>().Utf8Value();
  int64_t result = std::stoull(addr_str, 0, 16);
  char* addr = reinterpret_cast<char*>(result);
  int32_t length = info[1].As<Napi::Number>().Int32Value();

  static_assert(NODE_MAJOR_VERSION > 12, "nodejs version must > 12");
#if (NODE_RUNTIME_ELECTRON && NODE_MODULE_VERSION >= 109)
  // Because V8 sandboxed pointers was enabled since Electron 21, we have to
  // make a deep copy for Electron 21 and up.
  // See more details: https://www.electronjs.org/blog/v8-memory-cage
  Napi::ArrayBuffer array_buffer = Napi::ArrayBuffer::New(env, length);
  memcpy(array_buffer.Data(), addr, length);
  free(addr);
#else
  // For nodejs > 12 or electron < 21, we will create a new `BackingStore` and
  // take over the ownership of `addr`.
  // std::unique_ptr<v8::BackingStore> backing =
  // v8::ArrayBuffer::NewBackingStore(
  //     addr, length,
  //     [](void* data, size_t length, void* deleter_data) { free(data); },
  //     nullptr);
  // auto array_buffer =
  //     Napi::ArrayBuffer::New(env, std::move(backing));
  auto array_buffer = Napi::ArrayBuffer::New(
      env, addr, length, [](Napi::Env /*env*/, void* data) { free(data); });
#endif

  return array_buffer;
}

Napi::Value CreateArrayBufferCleaner(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  auto address = GetBufAddr(info[0]);
  int32_t offset = info[1].As<Napi::Number>().Int32Value();

  char* target = *reinterpret_cast<char**>(address + offset);
  return RclHandle::NewInstance(env, target, nullptr,
                                [](void* ptr) { free(ptr); });
}

Napi::Value setLoggerLevel(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int level = info[1].As<Napi::Number>().Int64Value();

  rcutils_ret_t ret = rcutils_logging_set_logger_level(name.c_str(), level);
  if (ret != RCUTILS_RET_OK) {
    Napi::Error::New(env, rcutils_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcutils_reset_error();
  }
  return env.Undefined();
}

Napi::Value GetLoggerEffectiveLevel(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int logger_level = rcutils_logging_get_logger_effective_level(name.c_str());

  if (logger_level < 0) {
    Napi::Error::New(env, rcutils_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcutils_reset_error();
    return env.Undefined();
  }
  return Napi::Number::New(env, logger_level);
}

Napi::Value GetNodeLoggerName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  const char* node_logger_name = rcl_node_get_logger_name(node);
  if (!node_logger_name) {
    return env.Undefined();
  }

  return Napi::String::New(env, node_logger_name);
}

Napi::Value Log(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int severity = info[1].As<Napi::Number>().Int64Value();
  std::string message = info[2].As<Napi::String>().Utf8Value();
  std::string function_name = info[3].As<Napi::String>().Utf8Value();
  size_t line_number = info[4].As<Napi::Number>().Int64Value();
  std::string file_name = info[5].As<Napi::String>().Utf8Value();
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);

  if (enabled) {
    rcutils_log_location_t logging_location = {function_name.c_str(),
                                               file_name.c_str(), line_number};
    rcutils_log(&logging_location, severity, name.c_str(), "%s",
                message.c_str());
  }

  return Napi::Boolean::New(env, enabled);
}

Napi::Value IsEnableFor(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int severity = info[1].As<Napi::Number>().Int64Value();
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);
  return Napi::Boolean::New(env, enabled);
}

Napi::Value CreateContext(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(malloc(sizeof(rcl_context_t)));
  *context = rcl_get_zero_initialized_context();
  auto js_obj =
      RclHandle::NewInstance(env, context, nullptr, [&env](void* ptr) {
        rcl_context_t* context = reinterpret_cast<rcl_context_t*>(ptr);
        rcl_ret_t ret = DestroyContext(env, context);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
      });

  return js_obj;
}

Napi::Value IsContextValid(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  bool is_valid = rcl_context_is_valid(context);
  return Napi::Boolean::New(env, is_valid);
}

void ExtractNamesAndTypes(rcl_names_and_types_t names_and_types,
                          Napi::Array* result_list) {
  Napi::Env env = result_list->Env();

  for (size_t i = 0; i < names_and_types.names.size; ++i) {
    Napi::Object item = Napi::Object::New(env);
    std::string topic_name = names_and_types.names.data[i];
    item.Set("name", Napi::String::New(env, names_and_types.names.data[i]));

    Napi::Array type_list =
        Napi::Array::New(env, names_and_types.types[i].size);
    for (size_t j = 0; j < names_and_types.types[i].size; ++j) {
      type_list.Set(j,
                    Napi::String::New(env, names_and_types.types[i].data[j]));
    }
    item.Set("types", type_list);
    result_list->Set(i, item);
  }
}

Napi::Value GetPublisherNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();
  bool no_demangle = info[3].As<Napi::Boolean>();

  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_get_publisher_names_and_types_by_node(
                               node, &allocator, no_demangle, node_name.c_str(),
                               node_namespace.c_str(), &topic_names_and_types),
                           "Failed to get_publisher_names_and_types.");

  Napi::Array result_list =
      Napi::Array::New(env, topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&topic_names_and_types),
                           "Failed to destroy topic_names_and_types");

  return result_list;
}

Napi::Value GetSubscriptionNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();
  bool no_demangle = info[3].As<Napi::Boolean>();

  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_get_subscriber_names_and_types_by_node(
                               node, &allocator, no_demangle, node_name.c_str(),
                               node_namespace.c_str(), &topic_names_and_types),
                           "Failed to get_publisher_names_and_types.");

  Napi::Array result_list =
      Napi::Array::New(env, topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&topic_names_and_types),
                           "Failed to destroy topic_names_and_types");

  return result_list;
}

Napi::Value GetServiceNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();

  rcl_names_and_types_t service_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_service_names_and_types_by_node(
          node, &allocator, node_name.c_str(), node_namespace.c_str(),
          &service_names_and_types),
      "Failed to get_service_names_and_types.");

  Napi::Array result_list =
      Napi::Array::New(env, service_names_and_types.names.size);
  ExtractNamesAndTypes(service_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_names_and_types_fini(&service_names_and_types),
      "Failed to destroy rcl_get_zero_initialized_names_and_types");

  return result_list;
}

Napi::Value GetClientNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();

  rcl_names_and_types_t client_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_service_names_and_types_by_node(
          node, &allocator, node_name.c_str(), node_namespace.c_str(),
          &client_names_and_types),
      "Failed to get_client_names_and_types.");

  Napi::Array result_list =
      Napi::Array::New(env, client_names_and_types.names.size);
  ExtractNamesAndTypes(client_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_names_and_types_fini(&client_names_and_types),
      "Failed to destroy rcl_get_zero_initialized_names_and_types");

  return result_list;
}

Napi::Value GetTopicNamesAndTypes(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  bool no_demangle = info[1].As<Napi::Boolean>();
  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_topic_names_and_types(node, &allocator, no_demangle,
                                    &topic_names_and_types),
      "Failed to get_publisher_names_and_types.");

  Napi::Array result_list =
      Napi::Array::New(env, topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&topic_names_and_types),
                           "Failed to destroy topic_names_and_types");

  return result_list;
}

Napi::Value GetServiceNamesAndTypes(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  rcl_names_and_types_t service_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_get_service_names_and_types(
                               node, &allocator, &service_names_and_types),
                           "Failed to get_publisher_names_and_types.");

  Napi::Array result_list =
      Napi::Array::New(env, service_names_and_types.names.size);
  ExtractNamesAndTypes(service_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&service_names_and_types),
                           "Failed to destroy topic_names_and_types");

  return result_list;
}

Napi::Value GetNodeNames(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  rcutils_string_array_t node_names =
      rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces =
      rcutils_get_zero_initialized_string_array();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_node_names(node, allocator, &node_names, &node_namespaces),
      "Failed to get_node_names.");

  Napi::Array result_list = Napi::Array::New(env, node_names.size);

  for (size_t i = 0; i < node_names.size; ++i) {
    Napi::Object item = Napi::Object::New(env);

    item.Set("name", Napi::String::New(env, node_names.data[i]));
    item.Set("namespace", Napi::String::New(env, node_namespaces.data[i]));

    result_list.Set(i, item);
  }

  rcutils_ret_t fini_names_ret = rcutils_string_array_fini(&node_names);
  rcutils_ret_t fini_namespaces_ret =
      rcutils_string_array_fini(&node_namespaces);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, fini_names_ret,
                           "Failed to destroy node_names");
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, fini_namespaces_ret,
                           "Failed to destroy node_namespaces");

  return result_list;
}

Napi::Value CountPublishers(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_count_publishers(node, topic_name.c_str(), &count),
      "Failed to count publishers.");

  return Napi::Number::New(env, static_cast<int32_t>(count));
}

Napi::Value CountSubscribers(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_count_subscribers(node, topic_name.c_str(), &count),
      "Failed to count subscribers.");

  return Napi::Number::New(env, static_cast<int32_t>(count));
}

Napi::Value ServiceServerIsAvailable(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  RclHandle* client_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(client_handle->ptr());

  bool is_available;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_service_server_is_available(node, client, &is_available),
      "Failed to get service state.");

  return Napi::Boolean::New(env, is_available);
}

Napi::Value PublishRawMessage(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  auto object = info[1].As<Napi::Buffer<char>>();
  rcl_serialized_message_t serialized_msg =
      rmw_get_zero_initialized_serialized_message();
  serialized_msg.buffer_capacity = object.Length();
  serialized_msg.buffer_length = serialized_msg.buffer_capacity;
  serialized_msg.buffer = reinterpret_cast<uint8_t*>(object.Data());

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_publish_serialized_message(publisher, &serialized_msg, nullptr),
      RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value RclTakeRaw(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  rcl_serialized_message_t msg = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_ret_t ret = rmw_serialized_message_init(&msg, 0u, &allocator);
  if (ret != RCL_RET_OK) {
    THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                             "Failed to deallocate message buffer.");
    return env.Undefined();
  }
  ret = rcl_take_serialized_message(subscription, &msg, nullptr, nullptr);
  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    rcl_reset_error();
    THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                             "Failed to deallocate message buffer.");
    return env.Undefined();
  }

  if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                             "Failed to deallocate message buffer.");
    return env.Undefined();
  }

  Napi::Buffer<char> buffer = Napi::Buffer<char>::Copy(
      env, reinterpret_cast<char*>(msg.buffer), msg.buffer_length);
  THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                           "Failed to deallocate message buffer");
  return buffer;
}

Napi::Value GetClientServiceName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* service_name = rcl_client_get_service_name(client);
  return Napi::String::New(env, service_name);
}

Napi::Value GetServiceServiceName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* service_name = rcl_service_get_service_name(service);
  return Napi::String::New(env, service_name);
}

Napi::Object InitBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("init", Napi::Function::New(env, InitRclnodejs));
  exports.Set("createNode", Napi::Function::New(env, CreateNode));
  exports.Set("getParameterOverrides",
              Napi::Function::New(env, GetParameterOverrides));
  exports.Set("createGuardCondition",
              Napi::Function::New(env, CreateGuardCondition));
  exports.Set("triggerGuardCondition",
              Napi::Function::New(env, TriggerGuardCondition));
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
  exports.Set("createClock", Napi::Function::New(env, CreateClock));
  exports.Set("clockGetNow", Napi::Function::New(env, ClockGetNow));
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
  exports.Set("rclTake", Napi::Function::New(env, RclTake));
  exports.Set("createSubscription",
              Napi::Function::New(env, CreateSubscription));
  exports.Set("hasContentFilter", Napi::Function::New(env, HasContentFilter));
  exports.Set("setContentFilter", Napi::Function::New(env, SetContentFilter));
  exports.Set("clearContentFilter",
              Napi::Function::New(env, ClearContentFilter));
  exports.Set("createPublisher", Napi::Function::New(env, CreatePublisher));
  exports.Set("publish", Napi::Function::New(env, Publish));
  exports.Set("getPublisherTopic", Napi::Function::New(env, GetPublisherTopic));
  exports.Set("getSubscriptionTopic",
              Napi::Function::New(env, GetSubscriptionTopic));
  exports.Set("createClient", Napi::Function::New(env, CreateClient));
  exports.Set("rclTakeResponse", Napi::Function::New(env, RclTakeResponse));
  exports.Set("sendRequest", Napi::Function::New(env, SendRequest));
  exports.Set("createService", Napi::Function::New(env, CreateService));
  exports.Set("rclTakeRequest", Napi::Function::New(env, RclTakeRequest));
  exports.Set("sendResponse", Napi::Function::New(env, SendResponse));
  exports.Set("shutdown", Napi::Function::New(env, Shutdown));
  exports.Set("validateFullTopicName",
              Napi::Function::New(env, ValidateFullTopicName));
  exports.Set("validateNodeName", Napi::Function::New(env, ValidateNodeName));
  exports.Set("validateTopicName", Napi::Function::New(env, ValidateTopicName));
  exports.Set("validateNamespace", Napi::Function::New(env, ValidateNamespace));
  exports.Set("expandTopicName", Napi::Function::New(env, ExpandTopicName));
  exports.Set("getNodeName", Napi::Function::New(env, GetNodeName));
  exports.Set("getNamespace", Napi::Function::New(env, GetNamespace));
  exports.Set("initString", Napi::Function::New(env, InitString));
  exports.Set("freeMemeoryAtOffset",
              Napi::Function::New(env, FreeMemeoryAtOffset));
  exports.Set("createArrayBufferFromAddress",
              Napi::Function::New(env, CreateArrayBufferFromAddress));
  exports.Set("createArrayBufferCleaner",
              Napi::Function::New(env, CreateArrayBufferCleaner));
  exports.Set("setLoggerLevel", Napi::Function::New(env, setLoggerLevel));
  exports.Set("getLoggerEffectiveLevel",
              Napi::Function::New(env, GetLoggerEffectiveLevel));
  exports.Set("getNodeLoggerName", Napi::Function::New(env, GetNodeLoggerName));
  exports.Set("log", Napi::Function::New(env, Log));
  exports.Set("isEnableFor", Napi::Function::New(env, IsEnableFor));
  exports.Set("createContext", Napi::Function::New(env, CreateContext));
  exports.Set("isContextValid", Napi::Function::New(env, IsContextValid));
  exports.Set("getPublisherNamesAndTypesByNode",
              Napi::Function::New(env, GetPublisherNamesAndTypesByNode));
  exports.Set("getSubscriptionNamesAndTypesByNode",
              Napi::Function::New(env, GetSubscriptionNamesAndTypesByNode));
  exports.Set("getServiceNamesAndTypesByNode",
              Napi::Function::New(env, GetServiceNamesAndTypesByNode));
  exports.Set("getClientNamesAndTypesByNode",
              Napi::Function::New(env, GetClientNamesAndTypesByNode));
  exports.Set("getTopicNamesAndTypes",
              Napi::Function::New(env, GetTopicNamesAndTypes));
  exports.Set("getServiceNamesAndTypes",
              Napi::Function::New(env, GetServiceNamesAndTypes));
  exports.Set("getNodeNames", Napi::Function::New(env, GetNodeNames));
  exports.Set("countPublishers", Napi::Function::New(env, CountPublishers));
  exports.Set("countSubscribers", Napi::Function::New(env, CountSubscribers));
  exports.Set("serviceServerIsAvailable",
              Napi::Function::New(env, ServiceServerIsAvailable));
  exports.Set("publishRawMessage", Napi::Function::New(env, PublishRawMessage));
  exports.Set("rclTakeRaw", Napi::Function::New(env, RclTakeRaw));
  exports.Set("getClientServiceName",
              Napi::Function::New(env, GetClientServiceName));
  exports.Set("getServiceServiceName",
              Napi::Function::New(env, GetServiceServiceName));
#if ROS_VERSION > 2205  // 2205 == Humble
  exports.Set("configureServiceIntrospection",
              Napi::Function::New(env, ConfigureServiceIntrospection));
#endif

  return exports;
}

// NODE_API_MODULE(rcl_bindings, Init)

}  // namespace rclnodejs
