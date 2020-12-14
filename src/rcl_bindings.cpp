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

#include <memory>
#include <string>
#include <vector>

#include "handle_manager.hpp"
#include "macros.hpp"
#include "rcl_handle.hpp"
#include "rcl_utilities.hpp"

namespace rclnodejs {

static v8::Local<v8::Object> wrapParameters(
    rcl_params_t* params);  // NOLINT(whitespace/line_length)

NAN_METHOD(Init) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_init_options_init(&init_options, allocator),
                           rcl_get_error_string().str);

  // preprocess Context
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  // preprocess argc & argv
  v8::Local<v8::Array> jsArgv = v8::Local<v8::Array>::Cast(info[1]);
  int argc = jsArgv->Length();
  char** argv = nullptr;
  if (argc > 0) {
    argv = reinterpret_cast<char**>(malloc(argc * sizeof(char*)));
    for (int i = 0; i < argc; i++) {
      Nan::MaybeLocal<v8::Value> jsElement = Nan::Get(jsArgv, i);
      Nan::Utf8String utf8_arg(jsElement.ToLocalChecked());
      int len = utf8_arg.length() + 1;
      argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char*)));
      snprintf(argv[i], len, "%s", *utf8_arg);
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
}

NAN_METHOD(GetParameterOverrides) {
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_arguments_t* parsed_args = &(context->global_arguments);
  rcl_params_t* params = NULL;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_arguments_get_param_overrides(parsed_args, &params),
      rcl_get_error_string().str);

  if (params == NULL) {
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  info.GetReturnValue().Set(wrapParameters(params));

  rcl_yaml_node_struct_fini(params);
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
static v8::Local<v8::Object> wrapParameters(rcl_params_t* parsed_args) {
  v8::Local<v8::Array> nodes = Nan::New<v8::Array>();

  // iterate over nodes
  for (size_t i = 0; i < parsed_args->num_nodes; i++) {
    v8::Local<v8::Object> node = Nan::New<v8::Object>();
    Nan::Set(node, Nan::New("name").ToLocalChecked(),
             Nan::New(parsed_args->node_names[i]).ToLocalChecked());

    rcl_node_params_t node_parameters = parsed_args->params[i];

    // iterate over node.parameters
    v8::Local<v8::Array> parameters = Nan::New<v8::Array>();
    for (size_t j = 0; j < node_parameters.num_params; j++) {
      v8::Local<v8::Object> parameter = Nan::New<v8::Object>();

      Nan::Set(
          parameter, Nan::New("name").ToLocalChecked(),
          Nan::New(parsed_args->params[i].parameter_names[j]).ToLocalChecked());

      int param_type = PARAMETER_NOT_SET;

      // for each value, find type & actual value
      rcl_variant_t value = node_parameters.parameter_values[j];
      if (value.bool_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BOOL;
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(),
                 (*value.bool_value ? Nan::True() : Nan::False()));
      } else if (value.integer_value != NULL) {  // NOLINT()
        param_type = PARAMETER_INTEGER;
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(),
                 Nan::New<v8::Number>(*value.integer_value));
      } else if (value.double_value != NULL) {  // NOLINT()
        param_type = PARAMETER_DOUBLE;
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(),
                 Nan::New<v8::Number>(*value.double_value));
      } else if (value.string_value != NULL) {  // NOLINT()
        param_type = PARAMETER_STRING;
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(),
                 Nan::New(value.string_value).ToLocalChecked());
      } else if (value.bool_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BOOL_ARRAY;
        v8::Local<v8::Array> bool_array = Nan::New<v8::Array>();

        for (size_t k = 0; k < value.bool_array_value->size; k++) {
          Nan::Set(
              bool_array, k,
              (value.bool_array_value->values[k] ? Nan::True() : Nan::False()));
        }
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(), bool_array);
      } else if (value.string_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_STRING_ARRAY;
        v8::Local<v8::Array> string_array = Nan::New<v8::Array>();
        for (size_t k = 0; k < value.string_array_value->size; k++) {
          Nan::Set(string_array, k,
                   Nan::New(value.string_array_value->data[k])
                       .ToLocalChecked());  // NOLINT(whitespace/line_length)
        }
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(), string_array);
      } else if (value.byte_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BYTE_ARRAY;
        v8::Local<v8::Array> byte_array = Nan::New<v8::Array>();
        for (size_t k = 0; k < value.byte_array_value->size; k++) {
          Nan::Set(byte_array, k, Nan::New(value.byte_array_value->values[k]));
        }
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(), byte_array);
      } else if (value.integer_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_INTEGER_ARRAY;
        v8::Local<v8::Array> int_array = Nan::New<v8::Array>();
        for (size_t k = 0; k < value.integer_array_value->size; k++) {
          Nan::Set(int_array, k,
                   Nan::New<v8::Number>(value.integer_array_value->values[k]));
        }
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(), int_array);
      } else if (value.double_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_DOUBLE_ARRAY;
        v8::Local<v8::Array> dbl_array = Nan::New<v8::Array>();
        for (size_t k = 0; k < value.double_array_value->size; k++) {
          Nan::Set(dbl_array, k,
                   Nan::New<v8::Number>(
                       value.double_array_value
                           ->values[k]));  // NOLINT(whitespace/line_length)
        }
        Nan::Set(parameter, Nan::New("value").ToLocalChecked(), dbl_array);
      }

      Nan::Set(parameter, Nan::New("type").ToLocalChecked(),
               Nan::New<v8::Number>(param_type));
      Nan::Set(parameters, j, parameter);
    }

    Nan::Set(node, Nan::New("parameters").ToLocalChecked(), parameters);
    Nan::Set(nodes, i, node);
  }

  return nodes;
}

NAN_METHOD(CreateNode) {
  std::string node_name(*Nan::Utf8String(info[0]));
  std::string name_space(*Nan::Utf8String(info[1]));
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[2]).ToLocalChecked());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(malloc(sizeof(rcl_node_t)));

  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_node_init(node, node_name.c_str(),
                                         name_space.c_str(), context, &options),
                           rcl_get_error_string().str);

  auto handle = RclHandle::NewInstance(node, nullptr, [](void* ptr) {
    rcl_node_t* node = reinterpret_cast<rcl_node_t*>(ptr);
    rcl_ret_t ret = rcl_node_fini(node);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  });
  info.GetReturnValue().Set(handle);
}

NAN_METHOD(CreateGuardCondition) {
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
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

  auto handle = RclHandle::NewInstance(gc, nullptr, [](void* ptr) {
    rcl_guard_condition_t* gc = reinterpret_cast<rcl_guard_condition_t*>(ptr);
    rcl_ret_t ret = rcl_guard_condition_fini(gc);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  });
  info.GetReturnValue().Set(handle);
}

NAN_METHOD(TriggerGuardCondition) {
  RclHandle* gc_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_guard_condition_t* gc =
      reinterpret_cast<rcl_guard_condition_t*>(gc_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_trigger_guard_condition(gc),
                           rcl_get_error_string().str);
}

NAN_METHOD(CreateTimer) {
  RclHandle* clock_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  int64_t period_ms = Nan::To<int64_t>(info[2]).FromJust();

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(malloc(sizeof(rcl_timer_t)));
  *timer = rcl_get_zero_initialized_timer();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_timer_init(timer, clock, context, RCL_MS_TO_NS(period_ms), nullptr,
                     rcl_get_default_allocator()),
      rcl_get_error_string().str);

  auto js_obj = RclHandle::NewInstance(timer, clock_handle, [](void* ptr) {
    rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(ptr);
    rcl_ret_t ret = rcl_timer_fini(timer);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  });
  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(IsTimerReady) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  bool is_ready = false;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_is_ready(timer, &is_ready),
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::New(is_ready));
}

NAN_METHOD(CallTimer) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_call(timer),
                           rcl_get_error_string().str);
}

NAN_METHOD(CancelTimer) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_cancel(timer),
                           rcl_get_error_string().str);
}

NAN_METHOD(IsTimerCanceled) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  bool is_canceled = false;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_is_canceled(timer, &is_canceled),
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::New(is_canceled));
}

NAN_METHOD(ResetTimer) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_reset(timer),
                           rcl_get_error_string().str);
}

NAN_METHOD(TimerGetTimeUntilNextCall) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  int64_t remaining_time = 0;

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_timer_get_time_until_next_call(timer, &remaining_time),
      rcl_get_error_string().str);

  info.GetReturnValue().Set(
      Nan::New<v8::String>(std::to_string(RCL_NS_TO_MS(remaining_time)))
          .ToLocalChecked());
}

NAN_METHOD(TimerGetTimeSinceLastCall) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  int64_t elapsed_time = 0;

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_timer_get_time_since_last_call(timer, &elapsed_time),
      rcl_get_error_string().str);

  info.GetReturnValue().Set(
      Nan::New<v8::String>(std::to_string(RCL_NS_TO_MS(elapsed_time)))
          .ToLocalChecked());
}

NAN_METHOD(CreateTimePoint) {
  std::string str(*Nan::Utf8String(info[0]));
  uint32_t clock_type = Nan::To<uint32_t>(info[1]).FromJust();
  rcl_time_point_t* time_point =
      reinterpret_cast<rcl_time_point_t*>(malloc(sizeof(rcl_time_point_t)));

  time_point->nanoseconds = std::stoll(str);
  time_point->clock_type = static_cast<rcl_clock_type_t>(clock_type);

  auto js_obj =
      RclHandle::NewInstance(time_point, nullptr, [](void* ptr) { free(ptr); });
  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(GetNanoseconds) {
  RclHandle* time_point_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_time_point_t* time_point =
      reinterpret_cast<rcl_time_point_t*>(time_point_handle->ptr());
  info.GetReturnValue().Set(
      Nan::New<v8::String>(std::to_string(time_point->nanoseconds))
          .ToLocalChecked());
}

NAN_METHOD(CreateDuration) {
  std::string str(*Nan::Utf8String(info[0]));
  rcl_duration_t* duration =
      reinterpret_cast<rcl_duration_t*>(malloc(sizeof(rcl_duration_t)));
  duration->nanoseconds = std::stoll(str);

  auto js_obj =
      RclHandle::NewInstance(duration, nullptr, [](void* ptr) { free(ptr); });
  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(GetDurationNanoseconds) {
  RclHandle* duration_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_duration_t* duration =
      reinterpret_cast<rcl_duration_t*>(duration_handle->ptr());

  info.GetReturnValue().Set(
      Nan::New<v8::String>(std::to_string(duration->nanoseconds))
          .ToLocalChecked());
}

NAN_METHOD(SetRosTimeOverrideIsEnabled) {
  RclHandle* clock_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());
  bool enabled = Nan::To<bool>(info[1]).FromJust();

  if (enabled) {
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_enable_ros_time_override(clock),
                             rcl_get_error_string().str);
  } else {
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_disable_ros_time_override(clock),
                             rcl_get_error_string().str);
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(SetRosTimeOverride) {
  RclHandle* clock_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());
  RclHandle* time_point_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_time_point_t* time_point =
      reinterpret_cast<rcl_time_point_t*>(time_point_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_set_ros_time_override(clock, time_point->nanoseconds),
      rcl_get_error_string().str);
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(GetRosTimeOverrideIsEnabled) {
  RclHandle* clock_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(clock_handle->ptr());

  bool is_enabled;
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_is_enabled_ros_time_override(clock, &is_enabled),
                           rcl_get_error_string().str);
  info.GetReturnValue().Set(Nan::New(is_enabled));
}

NAN_METHOD(CreateClock) {
  auto clock_type =
      static_cast<rcl_clock_type_t>(Nan::To<int32_t>(info[0]).FromJust());
  rcl_clock_t* clock =
      reinterpret_cast<rcl_clock_t*>(malloc(sizeof(rcl_clock_t)));
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_clock_init(clock_type, clock, &allocator),
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(
      RclHandle::NewInstance(clock, nullptr, [](void* ptr) {
        rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(ptr);
        rcl_ret_t ret = rcl_clock_fini(clock);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
      }));
}

static void ReturnJSTimeObj(
    Nan::NAN_METHOD_ARGS_TYPE info, int64_t nanoseconds,
    rcl_clock_type_t clock_type = RCL_CLOCK_UNINITIALIZED) {
  auto obj = v8::Object::New(v8::Isolate::GetCurrent());

  const auto sec = static_cast<std::int32_t>(RCL_NS_TO_S(nanoseconds));
  const auto nanosec =
      static_cast<std::int32_t>(nanoseconds % (1000 * 1000 * 1000));
  const int32_t type = clock_type;

  Nan::Set(obj, Nan::New("sec").ToLocalChecked(), Nan::New(sec));
  Nan::Set(obj, Nan::New("nanosec").ToLocalChecked(), Nan::New(nanosec));
  if (clock_type != RCL_CLOCK_UNINITIALIZED) {
    Nan::Set(obj, Nan::New("type").ToLocalChecked(), Nan::New(type));
  }

  info.GetReturnValue().Set(obj);
}

NAN_METHOD(ClockGetNow) {
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());
  rcl_time_point_t time_point;
  time_point.clock_type = clock->type;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_clock_get_now(clock, &time_point.nanoseconds),
                           rcl_get_error_string().str);

  ReturnJSTimeObj(info, time_point.nanoseconds, time_point.clock_type);
}

NAN_METHOD(StaticClockGetNow) {
  int32_t type = Nan::To<int32_t>(info[0]).FromJust();

  if (type < RCL_ROS_TIME && type > RCL_STEADY_TIME) {
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  rcl_clock_t ros_clock;
  rcl_time_point_t rcl_time;
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_clock_init(static_cast<rcl_clock_type_t>(type),
                                          &ros_clock, &allocator),
                           rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_clock_get_now(&ros_clock, &rcl_time.nanoseconds),
                           rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_clock_fini(&ros_clock),
                           rcl_get_error_string().str);

  ReturnJSTimeObj(info, rcl_time.nanoseconds, rcl_time.clock_type);
}

NAN_METHOD(TimeDiff) {
  int64_t s_sec = Nan::To<int32_t>(info[0]).FromJust();
  uint32_t s_nano = Nan::To<uint32_t>(info[1]).FromJust();
  int32_t s_type = Nan::To<int32_t>(info[2]).FromJust();

  int64_t f_sec = Nan::To<int32_t>(info[3]).FromJust();
  uint32_t f_nano = Nan::To<uint32_t>(info[4]).FromJust();
  int32_t f_type = Nan::To<int32_t>(info[5]).FromJust();

  rcl_time_point_t start;
  rcl_time_point_t finish;
  rcl_duration_t delta;

  start.nanoseconds = s_sec * 1000 * 1000 * 1000 + s_nano;
  start.clock_type = static_cast<rcl_clock_type_t>(s_type);

  finish.nanoseconds = f_sec * 1000 * 1000 * 1000 + f_nano;
  finish.clock_type = static_cast<rcl_clock_type_t>(f_type);

  auto ret = rcl_difference_times(&start, &finish, &delta);

  if (ret == RCL_RET_OK) {
    ReturnJSTimeObj(info, delta.nanoseconds);
    return;
  }

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(RclTake) {
  RclHandle* subscription_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());
  void* msg_taken =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_ret_t ret = rcl_take(subscription, msg_taken, nullptr, nullptr);

  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::False());
    return;
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    info.GetReturnValue().Set(Nan::True());
    return;
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(CreateSubscription) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string package_name(
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked()));
  std::string message_sub_folder(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));
  std::string message_name(
      *Nan::Utf8String(info[3]->ToString(currentContent).ToLocalChecked()));
  std::string topic(
      *Nan::Utf8String(info[4]->ToString(currentContent).ToLocalChecked()));

  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(malloc(sizeof(rcl_subscription_t)));
  *subscription = rcl_get_zero_initialized_subscription();

  rcl_subscription_options_t subscription_ops =
      rcl_subscription_get_default_options();
  auto qos_profile = GetQoSProfile(info[5]);

  if (qos_profile) {
    subscription_ops.qos = *qos_profile;
  }

  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);

  if (ts) {
    THROW_ERROR_IF_NOT_EQUAL(
        RCL_RET_OK,
        rcl_subscription_init(subscription, node, ts, topic.c_str(),
                              &subscription_ops),
        rcl_get_error_string().str);

    auto js_obj =
        RclHandle::NewInstance(subscription, node_handle, [node](void* ptr) {
          rcl_subscription_t* subscription =
              reinterpret_cast<rcl_subscription_t*>(ptr);
          rcl_ret_t ret = rcl_subscription_fini(subscription, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
        });
    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear().c_str());
  }
}

NAN_METHOD(CreatePublisher) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  // Extract arguments
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string package_name(
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked()));
  std::string message_sub_folder(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));
  std::string message_name(
      *Nan::Utf8String(info[3]->ToString(currentContent).ToLocalChecked()));
  std::string topic(
      *Nan::Utf8String(info[4]->ToString(currentContent).ToLocalChecked()));

  // Prepare publisher object
  rcl_publisher_t* publisher =
      reinterpret_cast<rcl_publisher_t*>(malloc(sizeof(rcl_publisher_t)));
  *publisher = rcl_get_zero_initialized_publisher();

  // Get type support object dynamically
  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);

  if (ts) {
    // Using default options
    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    auto qos_profile = GetQoSProfile(info[5]);

    if (qos_profile) {
      publisher_ops.qos = *qos_profile;
    }

    // Initialize the publisher
    THROW_ERROR_IF_NOT_EQUAL(
        rcl_publisher_init(publisher, node, ts, topic.c_str(), &publisher_ops),
        RCL_RET_OK, rcl_get_error_string().str);

    // Wrap the handle into JS object
    auto js_obj =
        RclHandle::NewInstance(publisher, node_handle, [node](void* ptr) {
          rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(ptr);
          rcl_ret_t ret = rcl_publisher_fini(publisher, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
        });

    // Everything is done
    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear().c_str());
  }
}

NAN_METHOD(Publish) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());

  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  THROW_ERROR_IF_NOT_EQUAL(rcl_publish(publisher, buffer, nullptr), RCL_RET_OK,
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(GetTopic) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());

  const char* topic = rcl_publisher_get_topic_name(publisher);
  info.GetReturnValue().Set(Nan::New(topic).ToLocalChecked());
}

NAN_METHOD(CreateClient) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string service_name(
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked()));
  std::string interface_name(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));
  std::string package_name(
      *Nan::Utf8String(info[3]->ToString(currentContent).ToLocalChecked()));

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
        RclHandle::NewInstance(client, node_handle, [node](void* ptr) {
          rcl_client_t* client = reinterpret_cast<rcl_client_t*>(ptr);
          rcl_ret_t ret = rcl_client_fini(client, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
        });

    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear().c_str());
  }
}

NAN_METHOD(SendRequest) {
  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());
  int64_t sequence_number;

  THROW_ERROR_IF_NOT_EQUAL(rcl_send_request(client, buffer, &sequence_number),
                           RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::New((uint32_t)sequence_number));
}

NAN_METHOD(RclTakeResponse) {
  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());
  int64_t sequence_number = Nan::To<int64_t>(info[1]).FromJust();

  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));
  header->sequence_number = sequence_number;

  void* taken_response =
      node::Buffer::Data(Nan::To<v8::Object>(info[2]).ToLocalChecked());
  rcl_ret_t ret = rcl_take_response(client, header, taken_response);
  free(header);

  if (ret == RCL_RET_OK) {
    info.GetReturnValue().Set(Nan::True());
    return;
  }

  rcl_reset_error();
  info.GetReturnValue().Set(Nan::False());
}

NAN_METHOD(CreateService) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string service_name(
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked()));
  std::string interface_name(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));
  std::string package_name(
      *Nan::Utf8String(info[3]->ToString(currentContent).ToLocalChecked()));

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
        RclHandle::NewInstance(service, node_handle, [node](void* ptr) {
          rcl_service_t* service = reinterpret_cast<rcl_service_t*>(ptr);
          rcl_ret_t ret = rcl_service_fini(service, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
        });

    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear().c_str());
  }
}

NAN_METHOD(RclTakeRequest) {
  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request =
      node::Buffer::Data(Nan::To<v8::Object>(info[2]).ToLocalChecked());
  rcl_ret_t ret = rcl_take_request(service, header, taken_request);
  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    auto js_obj =
        RclHandle::NewInstance(header, nullptr, [](void* ptr) { free(ptr); });
    info.GetReturnValue().Set(js_obj);
    return;
  }

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(GetServiceName) {
  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());

  const char* name = rcl_service_get_service_name(service);
  info.GetReturnValue().Set(Nan::New(name).ToLocalChecked());
}

NAN_METHOD(SendResponse) {
  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[1]).ToLocalChecked());

  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[2]).ToLocalChecked())
          ->ptr());

  THROW_ERROR_IF_NOT_EQUAL(rcl_send_response(service, header, buffer),
                           RCL_RET_OK, rcl_get_error_string().str);
}

NAN_METHOD(ValidateFullTopicName) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  int validation_result;
  size_t invalid_index;
  std::string topic_name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  rmw_ret_t ret = rmw_validate_full_topic_name(
      topic_name.c_str(), &validation_result, &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Nan::ThrowError(rmw_get_error_string().str);
    }
    rmw_reset_error();
    return info.GetReturnValue().Set(Nan::Undefined());
  }

  if (validation_result == RMW_NAMESPACE_VALID) {
    info.GetReturnValue().Set(Nan::Null());
    return;
  }
  const char* validation_message =
      rmw_full_topic_name_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  v8::Local<v8::Array> result_list = Nan::New<v8::Array>(2);
  Nan::Set(
      result_list, 0,
      Nan::New<v8::String>(std::string(validation_message)).ToLocalChecked());
  Nan::Set(result_list, 1, Nan::New((int32_t)invalid_index));

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(ValidateNodeName) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  int validation_result;
  size_t invalid_index;
  std::string node_name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  rmw_ret_t ret = rmw_validate_node_name(node_name.c_str(), &validation_result,
                                         &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Nan::ThrowError(rmw_get_error_string().str);
    }
    rmw_reset_error();
    return info.GetReturnValue().Set(Nan::Undefined());
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    info.GetReturnValue().Set(Nan::Null());
    return;
  }
  const char* validation_message =
      rmw_node_name_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  v8::Local<v8::Array> result_list = Nan::New<v8::Array>(2);
  Nan::Set(
      result_list, 0,
      Nan::New<v8::String>(std::string(validation_message)).ToLocalChecked());
  Nan::Set(result_list, 1, Nan::New((int32_t)invalid_index));

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(ValidateTopicName) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  int validation_result;
  size_t invalid_index;
  std::string topic_name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  rmw_ret_t ret = rcl_validate_topic_name(topic_name.c_str(),
                                          &validation_result, &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Nan::ThrowError(rmw_get_error_string().str);
    }
    rmw_reset_error();
    return info.GetReturnValue().Set(Nan::Undefined());
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    info.GetReturnValue().Set(Nan::Null());
    return;
  }
  const char* validation_message =
      rcl_topic_name_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  v8::Local<v8::Array> result_list = Nan::New<v8::Array>(2);
  Nan::Set(
      result_list, 0,
      Nan::New<v8::String>(std::string(validation_message)).ToLocalChecked());
  Nan::Set(result_list, 1, Nan::New((int32_t)invalid_index));

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(ValidateNamespace) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  int validation_result;
  size_t invalid_index;
  std::string namespace_name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  rmw_ret_t ret = rmw_validate_namespace(namespace_name.c_str(),
                                         &validation_result, &invalid_index);

  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      Nan::ThrowError(rmw_get_error_string().str);
    }
    rmw_reset_error();
    return info.GetReturnValue().Set(Nan::Undefined());
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    info.GetReturnValue().Set(Nan::Null());
    return;
  }
  const char* validation_message =
      rmw_namespace_validation_result_string(validation_result);
  THROW_ERROR_IF_EQUAL(nullptr, validation_message,
                       "Unable to get validation error message");

  v8::Local<v8::Array> result_list = Nan::New<v8::Array>(2);
  Nan::Set(
      result_list, 0,
      Nan::New<v8::String>(std::string(validation_message)).ToLocalChecked());
  Nan::Set(result_list, 1, Nan::New((int32_t)invalid_index));

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(ExpandTopicName) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  std::string topic_name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  std::string node_name(
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked()));
  std::string node_namespace(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));

  char* expanded_topic = nullptr;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_allocator_t rcutils_allocator = rcutils_get_default_allocator();
  rcutils_string_map_t substitutions_map =
      rcutils_get_zero_initialized_string_map();

  rcutils_ret_t rcutils_ret =
      rcutils_string_map_init(&substitutions_map, 0, rcutils_allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    if (rcutils_ret == RCUTILS_RET_BAD_ALLOC) {
      Nan::ThrowError(rcutils_get_error_string().str);
    }
    rcutils_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      Nan::ThrowError(rcl_get_error_string().str);
    }
    rcl_reset_error();

    rcutils_ret = rcutils_string_map_fini(&substitutions_map);
    if (rcutils_ret != RCUTILS_RET_OK) {
      Nan::ThrowError(rcutils_get_error_string().str);
      rcutils_reset_error();
    }
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  ret = rcl_expand_topic_name(topic_name.c_str(), node_name.c_str(),
                              node_namespace.c_str(), &substitutions_map,
                              allocator, &expanded_topic);

  rcutils_ret = rcutils_string_map_fini(&substitutions_map);
  if (rcutils_ret != RCUTILS_RET_OK) {
    Nan::ThrowError(rcutils_get_error_string().str);
    rcutils_reset_error();
    allocator.deallocate(expanded_topic, allocator.state);
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }
  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string().str);
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  if (!expanded_topic) {
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  rcl_allocator_t topic_allocator = rcl_get_default_allocator();
  std::string topic(expanded_topic);
  allocator.deallocate(expanded_topic, topic_allocator.state);
  info.GetReturnValue().Set(Nan::New<v8::String>(topic).ToLocalChecked());
}

NAN_METHOD(GetNodeName) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* node_name = rcl_node_get_name(node);
  if (!node_name) {
    info.GetReturnValue().Set(Nan::Undefined());
  } else {
    info.GetReturnValue().Set(Nan::New<v8::String>(node_name).ToLocalChecked());
  }
}

NAN_METHOD(GetNamespace) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* node_namespace = rcl_node_get_namespace(node);
  if (!node_namespace) {
    info.GetReturnValue().Set(Nan::Undefined());
  } else {
    info.GetReturnValue().Set(
        Nan::New<v8::String>(node_namespace).ToLocalChecked());
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
    v8::Local<v8::Object> object) {
  std::unique_ptr<rmw_qos_profile_t> qos_profile =
      std::make_unique<rmw_qos_profile_t>();

  auto history =
      Nan::Get(object, Nan::New("history").ToLocalChecked()).ToLocalChecked();
  auto depth =
      Nan::Get(object, Nan::New("depth").ToLocalChecked()).ToLocalChecked();
  auto reliability = Nan::Get(object, Nan::New("reliability").ToLocalChecked())
                         .ToLocalChecked();
  auto durability = Nan::Get(object, Nan::New("durability").ToLocalChecked())
                        .ToLocalChecked();
  auto avoid_ros_namespace_conventions =
      Nan::Get(object,
               Nan::New("avoidRosNameSpaceConventions").ToLocalChecked())
          .ToLocalChecked();

  qos_profile->history = static_cast<rmw_qos_history_policy_t>(
      Nan::To<uint32_t>(history).FromJust());
  qos_profile->depth = Nan::To<uint32_t>(depth).FromJust();
  qos_profile->reliability = static_cast<rmw_qos_reliability_policy_t>(
      Nan::To<uint32_t>(reliability).FromJust());
  qos_profile->durability = static_cast<rmw_qos_durability_policy_t>(
      Nan::To<uint32_t>(durability).FromJust());
  qos_profile->avoid_ros_namespace_conventions =
      Nan::To<bool>(avoid_ros_namespace_conventions).FromJust();

  return qos_profile;
}

std::unique_ptr<rmw_qos_profile_t> GetQoSProfile(v8::Local<v8::Value> qos) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  std::unique_ptr<rmw_qos_profile_t> qos_profile =
      std::make_unique<rmw_qos_profile_t>();

  if (qos->IsString()) {
    *qos_profile = *GetQoSProfileFromString(std::string(
        *Nan::Utf8String(qos->ToString(currentContent).ToLocalChecked())));
  } else if (qos->IsObject()) {
    qos_profile =
        GetQosProfileFromObject(Nan::To<v8::Object>(qos).ToLocalChecked());
  } else {
    return qos_profile;
  }
  return qos_profile;
}

int DestroyContext(rcl_context_t* context) {
  rcl_ret_t ret = RCL_RET_OK;
  if (context->impl) {
    if (rcl_context_is_valid(context)) {
      if (RCL_RET_OK != rcl_shutdown(context)) {
        Nan::ThrowError(rcl_get_error_string().str);
      }
      ret = rcl_context_fini(context);
    }
  }
  return ret;
}

NAN_METHOD(Shutdown) {
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  THROW_ERROR_IF_NOT_EQUAL(rcl_shutdown(context), RCL_RET_OK,
                           rcl_get_error_string().str);
  THROW_ERROR_IF_NOT_EQUAL(rcl_logging_fini(), RCL_RET_OK,
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(InitString) {
  void* buffer =
      node::Buffer::Data(Nan::To<v8::Object>(info[0]).ToLocalChecked());
#if ROS_VERSION >= 2006
  rosidl_runtime_c__String* ptr =
      reinterpret_cast<rosidl_runtime_c__String*>(buffer);

  rosidl_runtime_c__String__init(ptr);
#else
  rosidl_generator_c__String* ptr =
      reinterpret_cast<rosidl_generator_c__String*>(buffer);

  rosidl_generator_c__String__init(ptr);
#endif
  info.GetReturnValue().Set(Nan::Undefined());
}

inline char* GetBufAddr(v8::Local<v8::Value> buf) {
  return node::Buffer::Data(buf.As<v8::Object>());
}

NAN_METHOD(FreeMemeoryAtOffset) {
  v8::Local<v8::Value> buf = info[0];
  if (!node::Buffer::HasInstance(buf)) {
    return Nan::ThrowTypeError("Buffer instance expected as first argument");
  }

  int64_t offset =
      info[1]->IsNumber() ? Nan::To<int64_t>(info[1]).FromJust() : 0;
  auto ptr = GetBufAddr(buf) + offset;

  if (ptr == nullptr) {
    return Nan::ThrowError("Cannot read from NULL pointer");
  }

  char* val = *reinterpret_cast<char**>(ptr);
  free(val);
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(CreateArrayBufferFromAddress) {
  char* addr = GetBufAddr(info[0]);
  int32_t length = Nan::To<int32_t>(info[1]).FromJust();

  // We will create an ArrayBuffer with mode of
  // ArrayBufferCreationMode::kInternalized and copy data starting from |addr|,
  // thus the memory block will be collected by the garbage collector.
  v8::Local<v8::ArrayBuffer> array_buffer =
      v8::ArrayBuffer::New(v8::Isolate::GetCurrent(), addr, length,
                           v8::ArrayBufferCreationMode::kInternalized);

  info.GetReturnValue().Set(array_buffer);
}

NAN_METHOD(CreateArrayBufferCleaner) {
  auto address = GetBufAddr(info[0]);
  int32_t offset = Nan::To<int32_t>(info[1]).FromJust();

  char* target = *reinterpret_cast<char**>(address + offset);
  info.GetReturnValue().Set(
      RclHandle::NewInstance(target, nullptr, [](void* ptr) { free(ptr); }));
}

NAN_METHOD(setLoggerLevel) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  std::string name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  int level = Nan::To<int64_t>(info[1]).FromJust();

  rcutils_ret_t ret = rcutils_logging_set_logger_level(name.c_str(), level);
  if (ret != RCUTILS_RET_OK) {
    Nan::ThrowError(rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(GetLoggerEffectiveLevel) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  std::string name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  int logger_level = rcutils_logging_get_logger_effective_level(name.c_str());

  if (logger_level < 0) {
    Nan::ThrowError(rcutils_get_error_string().str);
    rcutils_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }
  info.GetReturnValue().Set(Nan::New(logger_level));
}

NAN_METHOD(GetNodeLoggerName) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  const char* node_logger_name = rcl_node_get_logger_name(node);
  if (!node_logger_name) {
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  info.GetReturnValue().Set(
      Nan::New<v8::String>(node_logger_name).ToLocalChecked());
}

NAN_METHOD(Log) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  std::string name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  int severity = Nan::To<int64_t>(info[1]).FromJust();
  std::string message(
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked()));
  std::string function_name(
      *Nan::Utf8String(info[3]->ToString(currentContent).ToLocalChecked()));
  size_t line_number = Nan::To<int64_t>(info[4]).FromJust();
  std::string file_name(
      *Nan::Utf8String(info[5]->ToString(currentContent).ToLocalChecked()));
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);

  if (enabled) {
    rcutils_log_location_t logging_location = {function_name.c_str(),
                                               file_name.c_str(), line_number};
    rcutils_log(&logging_location, severity, name.c_str(), "%s",
                message.c_str());
  }

  info.GetReturnValue().Set(Nan::New(enabled));
}

NAN_METHOD(IsEnableFor) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  std::string name(
      *Nan::Utf8String(info[0]->ToString(currentContent).ToLocalChecked()));
  int severity = Nan::To<int64_t>(info[1]).FromJust();
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);
  info.GetReturnValue().Set(Nan::New(enabled));
}

NAN_METHOD(CreateContext) {
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(malloc(sizeof(rcl_context_t)));
  *context = rcl_get_zero_initialized_context();
  auto js_obj = RclHandle::NewInstance(context, nullptr, [](void* ptr) {
    rcl_context_t* context = reinterpret_cast<rcl_context_t*>(ptr);
    rcl_ret_t ret = DestroyContext(context);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
  });

  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(IsContextValid) {
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  bool is_valid = rcl_context_is_valid(context);
  info.GetReturnValue().Set(Nan::New(is_valid));
}

void ExtractNamesAndTypes(rcl_names_and_types_t names_and_types,
                          v8::Local<v8::Array>* result_list) {
  for (size_t i = 0; i < names_and_types.names.size; ++i) {
    auto item = v8::Object::New(v8::Isolate::GetCurrent());
    std::string topic_name = names_and_types.names.data[i];
    Nan::Set(item, Nan::New("name").ToLocalChecked(),
             Nan::New(names_and_types.names.data[i]).ToLocalChecked());

    v8::Local<v8::Array> type_list =
        Nan::New<v8::Array>(names_and_types.types[i].size);
    for (size_t j = 0; j < names_and_types.types[i].size; ++j) {
      Nan::Set(type_list, j,
               Nan::New(names_and_types.types[i].data[j]).ToLocalChecked());
    }
    Nan::Set(item, Nan::New("types").ToLocalChecked(), type_list);
    Nan::Set(*result_list, i, item);
  }
}

NAN_METHOD(GetPublisherNamesAndTypesByNode) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name =
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked());
  std::string node_namespace =
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked());
  bool no_demangle = Nan::To<bool>(info[3]).FromJust();

  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_get_publisher_names_and_types_by_node(
                               node, &allocator, no_demangle, node_name.c_str(),
                               node_namespace.c_str(), &topic_names_and_types),
                           "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&topic_names_and_types),
                           "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetSubscriptionNamesAndTypesByNode) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name =
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked());
  std::string node_namespace =
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked());
  bool no_demangle = Nan::To<bool>(info[3]).FromJust();

  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_get_subscriber_names_and_types_by_node(
                               node, &allocator, no_demangle, node_name.c_str(),
                               node_namespace.c_str(), &topic_names_and_types),
                           "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&topic_names_and_types),
                           "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetServiceNamesAndTypesByNode) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name =
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked());
  std::string node_namespace =
      *Nan::Utf8String(info[2]->ToString(currentContent).ToLocalChecked());

  rcl_names_and_types_t service_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_service_names_and_types_by_node(
          node, &allocator, node_name.c_str(), node_namespace.c_str(),
          &service_names_and_types),
      "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(service_names_and_types.names.size);
  ExtractNamesAndTypes(service_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&service_names_and_types),
                           "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetTopicNamesAndTypes) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  bool no_demangle = Nan::To<bool>(info[1]).FromJust();
  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_topic_names_and_types(node, &allocator, no_demangle,
                                    &topic_names_and_types),
      "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&topic_names_and_types),
                           "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetServiceNamesAndTypes) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  rcl_names_and_types_t service_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_get_service_names_and_types(
                               node, &allocator, &service_names_and_types),
                           "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(service_names_and_types.names.size);
  ExtractNamesAndTypes(service_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&service_names_and_types),
                           "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetNodeNames) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
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

  v8::Local<v8::Array> result_list = Nan::New<v8::Array>(node_names.size);

  for (size_t i = 0; i < node_names.size; ++i) {
    auto item = v8::Object::New(v8::Isolate::GetCurrent());

    Nan::Set(item, Nan::New("name").ToLocalChecked(),
             Nan::New(node_names.data[i]).ToLocalChecked());
    Nan::Set(item, Nan::New("namespace").ToLocalChecked(),
             Nan::New(node_namespaces.data[i]).ToLocalChecked());

    Nan::Set(result_list, i, item);
  }

  rcutils_ret_t fini_names_ret = rcutils_string_array_fini(&node_names);
  rcutils_ret_t fini_namespaces_ret =
      rcutils_string_array_fini(&node_namespaces);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, fini_names_ret,
                           "Failed to destroy node_names");
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, fini_namespaces_ret,
                           "Failed to destroy node_namespaces");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(CountPublishers) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name =
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked());

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_count_publishers(node, topic_name.c_str(), &count),
      "Failed to count publishers.");

  v8::Local<v8::Integer> result =
      Nan::New<v8::Integer>(static_cast<int32_t>(count));
  info.GetReturnValue().Set(result);
}

NAN_METHOD(CountSubscribers) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name =
      *Nan::Utf8String(info[1]->ToString(currentContent).ToLocalChecked());

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_count_subscribers(node, topic_name.c_str(), &count),
      "Failed to count subscribers.");

  v8::Local<v8::Integer> result =
      Nan::New<v8::Integer>(static_cast<int32_t>(count));
  info.GetReturnValue().Set(result);
}

NAN_METHOD(ServiceServerIsAvailable) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  RclHandle* client_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(client_handle->ptr());

  bool is_available;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_service_server_is_available(node, client, &is_available),
      "Failed to get service state.");

  v8::Local<v8::Boolean> result = Nan::New<v8::Boolean>(is_available);
  info.GetReturnValue().Set(result);
}

NAN_METHOD(PublishRawMessage) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap<RclHandle>(
          Nan::To<v8::Object>(info[0]).ToLocalChecked())
          ->ptr());

  auto object = Nan::To<v8::Object>(info[1]).ToLocalChecked();
  rcl_serialized_message_t serialized_msg =
      rmw_get_zero_initialized_serialized_message();
  serialized_msg.buffer_capacity = node::Buffer::Length(object);
  serialized_msg.buffer_length = serialized_msg.buffer_capacity;
  serialized_msg.buffer =
      reinterpret_cast<uint8_t*>(node::Buffer::Data(object));

  THROW_ERROR_IF_NOT_EQUAL(
      rcl_publish_serialized_message(publisher, &serialized_msg, nullptr),
      RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(RclTakeRaw) {
  RclHandle* subscription_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  rcl_serialized_message_t msg = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_ret_t ret = rmw_serialized_message_init(&msg, 0u, &allocator);
  if (ret != RCL_RET_OK) {
    THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                             "Failed to deallocate message buffer.");
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }
  ret = rcl_take_serialized_message(subscription, &msg, nullptr, nullptr);
  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    rcl_reset_error();
    THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                             "Failed to deallocate message buffer.");
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                             "Failed to deallocate message buffer.");
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }

  info.GetReturnValue().Set(
      Nan::CopyBuffer(reinterpret_cast<char*>(msg.buffer), msg.buffer_length)
          .ToLocalChecked());
  THROW_ERROR_IF_NOT_EQUAL(rmw_serialized_message_fini(&msg), RCL_RET_OK,
                           "Failed to deallocate message buffer");
}

std::vector<BindingMethod> binding_methods = {
    {"init", Init},
    {"createNode", CreateNode},
    {"getParameterOverrides", GetParameterOverrides},
    {"createGuardCondition", CreateGuardCondition},
    {"triggerGuardCondition", TriggerGuardCondition},
    {"createTimer", CreateTimer},
    {"isTimerReady", IsTimerReady},
    {"callTimer", CallTimer},
    {"cancelTimer", CancelTimer},
    {"isTimerCanceled", IsTimerCanceled},
    {"resetTimer", ResetTimer},
    {"timerGetTimeSinceLastCall", TimerGetTimeSinceLastCall},
    {"timerGetTimeUntilNextCall", TimerGetTimeUntilNextCall},
    {"createClock", CreateClock},
    {"clockGetNow", ClockGetNow},
    {"staticClockGetNow", StaticClockGetNow},
    {"timeDiff", TimeDiff},
    {"createTimePoint", CreateTimePoint},
    {"getNanoseconds", GetNanoseconds},
    {"createDuration", CreateDuration},
    {"getDurationNanoseconds", GetDurationNanoseconds},
    {"setRosTimeOverrideIsEnabled", SetRosTimeOverrideIsEnabled},
    {"setRosTimeOverride", SetRosTimeOverride},
    {"getRosTimeOverrideIsEnabled", GetRosTimeOverrideIsEnabled},
    {"rclTake", RclTake},
    {"createSubscription", CreateSubscription},
    {"createPublisher", CreatePublisher},
    {"publish", Publish},
    {"getTopic", GetTopic},
    {"createClient", CreateClient},
    {"rclTakeResponse", RclTakeResponse},
    {"sendRequest", SendRequest},
    {"createService", CreateService},
    {"getServiceName", GetServiceName},
    {"rclTakeRequest", RclTakeRequest},
    {"sendResponse", SendResponse},
    {"shutdown", Shutdown},
    {"validateFullTopicName", ValidateFullTopicName},
    {"validateNodeName", ValidateNodeName},
    {"validateTopicName", ValidateTopicName},
    {"validateNamespace", ValidateNamespace},
    {"expandTopicName", ExpandTopicName},
    {"getNodeName", GetNodeName},
    {"getNamespace", GetNamespace},
    {"initString", InitString},
    {"freeMemeoryAtOffset", FreeMemeoryAtOffset},
    {"createArrayBufferFromAddress", CreateArrayBufferFromAddress},
    {"createArrayBufferCleaner", CreateArrayBufferCleaner},
    {"setLoggerLevel", setLoggerLevel},
    {"getLoggerEffectiveLevel", GetLoggerEffectiveLevel},
    {"getNodeLoggerName", GetNodeLoggerName},
    {"log", Log},
    {"isEnableFor", IsEnableFor},
    {"createContext", CreateContext},
    {"isContextValid", IsContextValid},
    {"getPublisherNamesAndTypesByNode", GetPublisherNamesAndTypesByNode},
    {"getSubscriptionNamesAndTypesByNode", GetSubscriptionNamesAndTypesByNode},
    {"getServiceNamesAndTypesByNode", GetServiceNamesAndTypesByNode},
    {"getTopicNamesAndTypes", GetTopicNamesAndTypes},
    {"getServiceNamesAndTypes", GetServiceNamesAndTypes},
    {"getNodeNames", GetNodeNames},
    {"countPublishers", CountPublishers},
    {"countSubscribers", CountSubscribers},
    {"serviceServerIsAvailable", ServiceServerIsAvailable},
    {"publishRawMessage", PublishRawMessage},
    {"rclTakeRaw", RclTakeRaw},
    {"", nullptr}};

}  // namespace rclnodejs
