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

#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/graph.h>
#include <rcl/node.h>
#include <rcl/rcl.h>
#include <rcl/validate_topic_name.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>
#include <rosidl_generator_c/string_functions.h>

#include <memory>
#include <string>

#include "handle_manager.hpp"
#include "macros.hpp"
#include "rcl_handle.hpp"
#include "rcl_utilities.hpp"
#include "shadow_node.hpp"

namespace rclnodejs {

std::unique_ptr<rmw_qos_profile_t> GetQoSProfile(v8::Local<v8::Value> qos);

NAN_METHOD(Init) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);

  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_init(0, nullptr, &init_options, context),
                           rcl_get_error_string().str);
}

NAN_METHOD(CreateNode) {
  std::string node_name(*v8::String::Utf8Value(info[0]));
  std::string name_space(*v8::String::Utf8Value(info[1]));
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(info[2]->ToObject());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(malloc(sizeof(rcl_node_t)));

  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_node_init(node, node_name.c_str(), name_space.c_str(), context,
          &options),
      rcl_get_error_string().str);

  auto handle = RclHandle::NewInstance(node, nullptr,
                                       [node] { return rcl_node_fini(node); });
  info.GetReturnValue().Set(handle);
}

NAN_METHOD(CreateTimer) {
  int64_t period_ms = info[0]->IntegerValue();
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(info[1]->ToObject());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(malloc(sizeof(rcl_timer_t)));
  *timer = rcl_get_zero_initialized_timer();
  rcl_clock_t* clock =
      reinterpret_cast<rcl_clock_t*>(malloc(sizeof(rcl_clock_t)));
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_clock_init(RCL_STEADY_TIME,
                                          clock,
                                          &allocator),
                           rcl_get_error_string().str);
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_init(timer, clock, context,
                              RCL_MS_TO_NS(period_ms), nullptr,
                                  rcl_get_default_allocator()),
                           rcl_get_error_string().str);

  auto js_obj = RclHandle::NewInstance(
      timer, nullptr, [timer, clock] {
        rcl_ret_t ret_clock = rcl_clock_fini(clock);
        free(clock);
        rcl_ret_t ret_timer = rcl_timer_fini(timer);
        return ret_clock || ret_timer;
      });
  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(IsTimerReady) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  bool is_ready = false;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_is_ready(timer, &is_ready),
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::New(is_ready));
}

NAN_METHOD(CallTimer) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_call(timer),
                           rcl_get_error_string().str);
}

NAN_METHOD(CancelTimer) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_cancel(timer),
                           rcl_get_error_string().str);
}

NAN_METHOD(IsTimerCanceled) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());
  bool is_canceled = false;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_timer_is_canceled(timer, &is_canceled),
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::New(is_canceled));
}

NAN_METHOD(ResetTimer) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, rcl_timer_reset(timer),
                           rcl_get_error_string().str);
}

NAN_METHOD(TimerGetTimeUntilNextCall) {
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
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
  RclHandle* timer_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
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
  std::string str(*v8::String::Utf8Value(info[0]));
  uint32_t clock_type = info[1]->Uint32Value();
  rcl_time_point_t* time_point =
      reinterpret_cast<rcl_time_point_t*>(malloc(sizeof(rcl_time_point_t)));

  time_point->nanoseconds = std::stoll(str);
  time_point->clock_type = static_cast<rcl_clock_type_t>(clock_type);

  auto js_obj = RclHandle::NewInstance(
      time_point, nullptr, nullptr);
  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(GetNanoseconds) {
  RclHandle* time_point_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_time_point_t* time_point = reinterpret_cast<rcl_time_point_t*>(
      time_point_handle->ptr());
  info.GetReturnValue().Set(Nan::New<v8::String>(
      std::to_string(time_point->nanoseconds)).ToLocalChecked());
}

NAN_METHOD(CreateDuration) {
  std::string str(*v8::String::Utf8Value(info[0]));
  rcl_duration_t* duration =
      reinterpret_cast<rcl_duration_t*>(malloc(sizeof(rcl_duration_t)));
  duration->nanoseconds = std::stoll(str);

  auto js_obj = RclHandle::NewInstance(
      duration, nullptr, nullptr);
  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(GetDurationNanoseconds) {
    RclHandle* duration_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_duration_t* duration = reinterpret_cast<rcl_duration_t*>(
      duration_handle->ptr());

  info.GetReturnValue().Set(Nan::New<v8::String>(
      std::to_string(duration->nanoseconds)).ToLocalChecked());
}

NAN_METHOD(SetRosTimeOverrideIsEnabled) {
  RclHandle* clock_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      clock_handle->ptr());
  bool enabled = Nan::To<bool>(info[1]).FromJust();

  if (enabled) {
    THROW_ERROR_IF_NOT_EQUAL(
        RCL_RET_OK, rcl_enable_ros_time_override(clock),
        rcl_get_error_string().str);
  } else {
    THROW_ERROR_IF_NOT_EQUAL(
        RCL_RET_OK, rcl_disable_ros_time_override(clock),
        rcl_get_error_string().str);
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(SetRosTimeOverride) {
  RclHandle* clock_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      clock_handle->ptr());
  RclHandle* time_point_handle =
      RclHandle::Unwrap<RclHandle>(info[1]->ToObject());
  rcl_time_point_t* time_point = reinterpret_cast<rcl_time_point_t*>(
      time_point_handle->ptr());

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_set_ros_time_override(clock, time_point->nanoseconds),
      rcl_get_error_string().str);
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(GetRosTimeOverrideIsEnabled) {
  RclHandle* clock_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      clock_handle->ptr());

  bool is_enabled;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_is_enabled_ros_time_override(clock, &is_enabled),
      rcl_get_error_string().str);
  info.GetReturnValue().Set(Nan::New(is_enabled));
}

NAN_METHOD(CreateClock) {
  auto clock_type = static_cast<rcl_clock_type_t>(Nan::To<int32_t>(
      info[0]).FromJust());
  rcl_clock_t* clock =
      reinterpret_cast<rcl_clock_t*>(malloc(sizeof(rcl_clock_t)));
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
      rcl_clock_init(clock_type, clock, &allocator),
      rcl_get_error_string().str);

  info.GetReturnValue().Set(RclHandle::NewInstance(clock,
      nullptr,
      [clock]() {
        return rcl_clock_fini(clock);
      }));
}

static void ReturnJSTimeObj(Nan::NAN_METHOD_ARGS_TYPE info,
    int64_t nanoseconds,
    rcl_clock_type_t clock_type = RCL_CLOCK_UNINITIALIZED) {
  auto obj = v8::Object::New(v8::Isolate::GetCurrent());

  const auto sec = static_cast<std::int32_t>(
      RCL_NS_TO_S(nanoseconds));
  const auto nanosec = static_cast<std::int32_t>(
      nanoseconds % (1000 * 1000 * 1000));
  const int32_t type = clock_type;

  obj->Set(Nan::New("sec").ToLocalChecked(), Nan::New(sec));
  obj->Set(Nan::New("nanosec").ToLocalChecked(), Nan::New(nanosec));
  if (clock_type != RCL_CLOCK_UNINITIALIZED) {
    obj->Set(Nan::New("type").ToLocalChecked(), Nan::New(type));
  }

  info.GetReturnValue().Set(obj);
}

NAN_METHOD(ClockGetNow) {
  rcl_clock_t* clock = reinterpret_cast<rcl_clock_t*>(
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject())->ptr());
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
                     &ros_clock,
                     &allocator),
      rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
      rcl_clock_get_now(&ros_clock, &rcl_time.nanoseconds),
      rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
      rcl_clock_fini(&ros_clock),
      rcl_get_error_string().str);

  ReturnJSTimeObj(info, rcl_time.nanoseconds, rcl_time.clock_type);
}

NAN_METHOD(TimeDiff) {
  int64_t s_sec  = Nan::To<int32_t>(info[0]).FromJust();
  uint32_t s_nano = Nan::To<uint32_t>(info[1]).FromJust();
  int32_t s_type = Nan::To<int32_t>(info[2]).FromJust();

  int64_t f_sec  = Nan::To<int32_t>(info[3]).FromJust();
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
  RclHandle* subscription_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());
  void* msg_taken = node::Buffer::Data(info[1]->ToObject());
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
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string package_name(*Nan::Utf8String(info[1]->ToString()));
  std::string message_sub_folder(*Nan::Utf8String(info[2]->ToString()));
  std::string message_name(*Nan::Utf8String(info[3]->ToString()));
  std::string topic(*Nan::Utf8String(info[4]->ToString()));

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
        RclHandle::NewInstance(subscription, node_handle, [subscription, node] {
          return rcl_subscription_fini(subscription, node);
        });
    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear());
  }
}

NAN_METHOD(CreatePublisher) {
  // Extract arguments
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string package_name(*Nan::Utf8String(info[1]->ToString()));
  std::string message_sub_folder(*Nan::Utf8String(info[2]->ToString()));
  std::string message_name(*Nan::Utf8String(info[3]->ToString()));
  std::string topic(*Nan::Utf8String(info[4]->ToString()));

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
    auto js_obj = RclHandle::NewInstance(
        publisher, node_handle,
        [publisher, node]() { return rcl_publisher_fini(publisher, node); });

    // Everything is done
    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear());
  }
}

NAN_METHOD(Publish) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject())->ptr());

  void* buffer = node::Buffer::Data(info[1]->ToObject());
  THROW_ERROR_IF_NOT_EQUAL(rcl_publish(publisher, buffer, nullptr), RCL_RET_OK,
                           rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(CreateClient) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string service_name(*Nan::Utf8String(info[1]->ToString()));
  std::string interface_name(*Nan::Utf8String(info[2]->ToString()));
  std::string package_name(*Nan::Utf8String(info[3]->ToString()));

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

    auto js_obj = RclHandle::NewInstance(client, node_handle, [client, node] {
      return rcl_client_fini(client, node);
    });

    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear());
  }
}

NAN_METHOD(SendRequest) {
  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject())->ptr());
  void* buffer = node::Buffer::Data(info[1]->ToObject());
  int64_t sequence_number;

  THROW_ERROR_IF_NOT_EQUAL(rcl_send_request(client, buffer, &sequence_number),
                           RCL_RET_OK, rcl_get_error_string().str);

  info.GetReturnValue().Set(Nan::New((uint32_t)sequence_number));
}

NAN_METHOD(RclTakeResponse) {
  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject())->ptr());
  int64_t sequence_number = info[1]->IntegerValue();

  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));
  header->sequence_number = sequence_number;

  void* taken_response = node::Buffer::Data(info[2]->ToObject());
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
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string service_name(*Nan::Utf8String(info[1]->ToString()));
  std::string interface_name(*Nan::Utf8String(info[2]->ToString()));
  std::string package_name(*Nan::Utf8String(info[3]->ToString()));

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
    auto js_obj = RclHandle::NewInstance(service, node_handle, [service, node] {
      return rcl_service_fini(service, node);
    });

    info.GetReturnValue().Set(js_obj);
  } else {
    Nan::ThrowError(GetErrorMessageAndClear());
  }
}

NAN_METHOD(RclTakeRequest) {
  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject())->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request = node::Buffer::Data(info[2]->ToObject());
  rcl_ret_t ret = rcl_take_request(service, header, taken_request);
  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    auto js_obj = RclHandle::NewInstance(header);
    info.GetReturnValue().Set(js_obj);
    return;
  }

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(SendResponse) {
  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject())->ptr());
  void* buffer = node::Buffer::Data(info[1]->ToObject());

  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap<RclHandle>(info[2]->ToObject())->ptr());

  THROW_ERROR_IF_NOT_EQUAL(rcl_send_response(service, header, buffer),
                           RCL_RET_OK, rcl_get_error_string().str);
}

NAN_METHOD(ValidateFullTopicName) {
  int validation_result;
  size_t invalid_index;
  std::string topic_name(*Nan::Utf8String(info[0]->ToString()));
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
  int validation_result;
  size_t invalid_index;
  std::string node_name(*Nan::Utf8String(info[0]->ToString()));
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
  int validation_result;
  size_t invalid_index;
  std::string topic_name(*Nan::Utf8String(info[0]->ToString()));
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
  int validation_result;
  size_t invalid_index;
  std::string namespace_name(*Nan::Utf8String(info[0]->ToString()));
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
  std::string topic_name(*Nan::Utf8String(info[0]->ToString()));
  std::string node_name(*Nan::Utf8String(info[1]->ToString()));
  std::string node_namespace(*Nan::Utf8String(info[2]->ToString()));

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
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* node_name = rcl_node_get_name(node);
  if (!node_name) {
    info.GetReturnValue().Set(Nan::Undefined());
  } else {
    info.GetReturnValue().Set(Nan::New<v8::String>(node_name).ToLocalChecked());
  }
}

NAN_METHOD(GetNamespace) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
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
  } else {
    return &rmw_qos_profile_default;;
  }

  return qos_profile;
}

std::unique_ptr<rmw_qos_profile_t> GetQosProfileFromObject(
    v8::Local<v8::Object> object) {
  std::unique_ptr<rmw_qos_profile_t> qos_profile =
      std::make_unique<rmw_qos_profile_t>();

  qos_profile->history = static_cast<rmw_qos_history_policy_t>(
      object->Get(Nan::New("history").ToLocalChecked())->Uint32Value());
  qos_profile->depth =
      object->Get(Nan::New("depth").ToLocalChecked())->Uint32Value();
  qos_profile->reliability = static_cast<rmw_qos_reliability_policy_t>(
      object->Get(Nan::New("reliability").ToLocalChecked())->Uint32Value());
  qos_profile->durability = static_cast<rmw_qos_durability_policy_t>(
      object->Get(Nan::New("durability").ToLocalChecked())->Uint32Value());
  qos_profile->avoid_ros_namespace_conventions =
      object->Get(Nan::New("avoidRosNameSpaceConventions").ToLocalChecked())
          ->BooleanValue();

  return qos_profile;
}

std::unique_ptr<rmw_qos_profile_t> GetQoSProfile(v8::Local<v8::Value> qos) {
  std::unique_ptr<rmw_qos_profile_t> qos_profile =
      std::make_unique<rmw_qos_profile_t>();

  if (qos->IsString()) {
    *qos_profile = *GetQoSProfileFromString(
        std::string(*Nan::Utf8String(qos->ToString())));
  } else if (qos->IsObject()) {
    qos_profile = GetQosProfileFromObject(qos->ToObject());
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
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  THROW_ERROR_IF_NOT_EQUAL(rcl_shutdown(context), RCL_RET_OK,
                           rcl_get_error_string().str);
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(InitString) {
  void* buffer = node::Buffer::Data(info[0]->ToObject());
  rosidl_generator_c__String* ptr =
      reinterpret_cast<rosidl_generator_c__String*>(buffer);

  rosidl_generator_c__String__init(ptr);
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
  auto address = GetBufAddr(info[0]);
  int32_t length = Nan::To<int32_t>(info[1]).FromJust();

  auto array_buffer =
      v8::ArrayBuffer::New(v8::Isolate::GetCurrent(), address, length,
                           v8::ArrayBufferCreationMode::kExternalized);

  info.GetReturnValue().Set(array_buffer);
}

NAN_METHOD(CreateArrayBufferCleaner) {
  auto address = GetBufAddr(info[0]);
  int32_t offset = Nan::To<int32_t>(info[1]).FromJust();

  char* target = *reinterpret_cast<char**>(address + offset);
  info.GetReturnValue().Set(
      RclHandle::NewInstance(target, nullptr, [] { return RCL_RET_OK; }));
}

NAN_METHOD(setLoggerLevel) {
  std::string name(*Nan::Utf8String(info[0]->ToString()));
  int level = info[1]->IntegerValue();

  rcutils_ret_t ret = rcutils_logging_set_logger_level(name.c_str(), level);
  if (ret != RCUTILS_RET_OK) {
    Nan::ThrowError(rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(GetLoggerEffectiveLevel) {
  std::string name(*Nan::Utf8String(info[0]->ToString()));
  int logger_level = rcutils_logging_get_logger_effective_level(name.c_str());

  if (logger_level < 0) {
    Nan::ThrowError(rcutils_get_error_string().str);
    rcutils_reset_error();
    info.GetReturnValue().Set(Nan::Undefined());
    return;
  }
  info.GetReturnValue().Set(Nan::New(logger_level));
}

NAN_METHOD(Log) {
  std::string name(*Nan::Utf8String(info[0]->ToString()));
  int severity = info[1]->IntegerValue();
  std::string message(*Nan::Utf8String(info[2]->ToString()));
  std::string function_name(*Nan::Utf8String(info[3]->ToString()));
  size_t line_number = info[4]->IntegerValue();
  std::string file_name(*Nan::Utf8String(info[5]->ToString()));
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);

  if (enabled) {
    rcutils_log_location_t logging_location = {function_name.c_str(),
                                               file_name.c_str(),
                                               line_number};
    rcutils_log(&logging_location, severity, name.c_str(), message.c_str());
  }

  info.GetReturnValue().Set(Nan::New(enabled));
}

NAN_METHOD(IsEnableFor) {
  std::string name(*Nan::Utf8String(info[0]->ToString()));
  int severity = info[1]->IntegerValue();
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);
  info.GetReturnValue().Set(Nan::New(enabled));
}

NAN_METHOD(CreateContext) {
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(malloc(sizeof(rcl_context_t)));
  *context = rcl_get_zero_initialized_context();
  auto js_obj = RclHandle::NewInstance(context, nullptr,
      std::bind(DestroyContext, context));

  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(IsContextValid) {
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  bool is_valid = rcl_context_is_valid(context);
  info.GetReturnValue().Set(Nan::New(is_valid));
}

void ExtractNamesAndTypes(rcl_names_and_types_t names_and_types,
                          v8::Local<v8::Array>* result_list) {
  for (int i = 0; i < names_and_types.names.size; ++i) {
    auto item = v8::Object::New(v8::Isolate::GetCurrent());
    std::string topic_name = names_and_types.names.data[i];
    item->Set(Nan::New("name").ToLocalChecked(),
              Nan::New(names_and_types.names.data[i]).ToLocalChecked());

    v8::Local<v8::Array> type_list =
      Nan::New<v8::Array>(names_and_types.types[i].size);
    for (int j = 0; j < names_and_types.types[i].size; ++j) {
      Nan::Set(
        type_list, j,
        Nan::New(names_and_types.types[i].data[j]).ToLocalChecked());
    }
    item->Set(Nan::New("types").ToLocalChecked(), type_list);
    Nan::Set(*result_list, i, item);
  }
}

NAN_METHOD(GetPublisherNamesAndTypesByNode) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = *Nan::Utf8String(info[1]->ToString());
  std::string node_namespace = *Nan::Utf8String(info[2]->ToString());
  bool no_demangle = Nan::To<bool>(info[3]).FromJust();

  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_publisher_names_and_types_by_node(node, &allocator, no_demangle,
          node_name.c_str(), node_namespace.c_str(), &topic_names_and_types),
      "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_names_and_types_fini(&topic_names_and_types),
      "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetSubscriptionNamesAndTypesByNode) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = *Nan::Utf8String(info[1]->ToString());
  std::string node_namespace = *Nan::Utf8String(info[2]->ToString());
  bool no_demangle = Nan::To<bool>(info[3]).FromJust();

  rcl_names_and_types_t topic_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_subscriber_names_and_types_by_node(node, &allocator, no_demangle,
          node_name.c_str(), node_namespace.c_str(), &topic_names_and_types),
      "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(topic_names_and_types.names.size);
  ExtractNamesAndTypes(topic_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_names_and_types_fini(&topic_names_and_types),
      "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetServiceNamesAndTypesByNode) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = *Nan::Utf8String(info[1]->ToString());
  std::string node_namespace = *Nan::Utf8String(info[2]->ToString());

  rcl_names_and_types_t service_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_get_service_names_and_types_by_node(node, &allocator,
          node_name.c_str(), node_namespace.c_str(), &service_names_and_types),
      "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(service_names_and_types.names.size);
  ExtractNamesAndTypes(service_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_names_and_types_fini(&service_names_and_types),
      "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetTopicNamesAndTypes) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
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

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_names_and_types_fini(&topic_names_and_types),
      "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

NAN_METHOD(GetServiceNamesAndTypes) {
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  rcl_names_and_types_t service_names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  THROW_ERROR_IF_NOT_EQUAL(
    RCL_RET_OK,
    rcl_get_service_names_and_types(node, &allocator, &service_names_and_types),
    "Failed to get_publisher_names_and_types.");

  v8::Local<v8::Array> result_list =
      Nan::New<v8::Array>(service_names_and_types.names.size);
  ExtractNamesAndTypes(service_names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_names_and_types_fini(&service_names_and_types),
      "Failed to destroy topic_names_and_types");

  info.GetReturnValue().Set(result_list);
}

uint32_t GetBindingMethodsCount(BindingMethod* methods) {
  uint32_t count = 0;
  while (methods[count].function) {
    count++;
  }
  return count;
}

BindingMethod binding_methods[] = {
    {"init", Init},
    {"createNode", CreateNode},
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
    {"createClient", CreateClient},
    {"rclTakeResponse", RclTakeResponse},
    {"sendRequest", SendRequest},
    {"createService", CreateService},
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
    {"log", Log},
    {"isEnableFor", IsEnableFor},
    {"createContext", CreateContext},
    {"ok", IsContextValid},
    {"getPublisherNamesAndTypesByNode", GetPublisherNamesAndTypesByNode},
    {"getSubscriptionNamesAndTypesByNode", GetSubscriptionNamesAndTypesByNode},
    {"getServiceNamesAndTypesByNode", GetServiceNamesAndTypesByNode},
    {"getTopicNamesAndTypes", GetTopicNamesAndTypes},
    {"getServiceNamesAndTypes", GetServiceNamesAndTypes},
    {"", nullptr}};

}  // namespace rclnodejs
