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

#include "rcl_event_handle_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include "rcl_handle.h"

namespace {

typedef union event_callback_data {
  // Subscription events
  rmw_requested_deadline_missed_status_t requested_deadline_missed;
  rmw_liveliness_changed_status_t liveliness_changed;
  rmw_message_lost_status_t message_lost;
  rmw_requested_qos_incompatible_event_status_t requested_incompatible_qos;
  rmw_matched_status_t subscription_matched;
  // Publisher events
  rmw_offered_deadline_missed_status_t offered_deadline_missed;
  rmw_liveliness_lost_status_t liveliness_lost;
  rmw_offered_qos_incompatible_event_status_t offered_incompatible_qos;
  rmw_matched_status_t publisher_matched;

  rmw_incompatible_type_status_t incompatible_type;
} event_callback_data_t;

rcl_event_t* CreateEventHandle() {
  rcl_event_t* event =
      reinterpret_cast<rcl_event_t*>(malloc(sizeof(rcl_event_t)));
  *event = rcl_get_zero_initialized_event();
  return event;
}

Napi::Value CreateJSObjectForSubscriptionEvent(
    Napi::Env env, rcl_subscription_event_type_t subscription_event_type,
    const event_callback_data_t& data) {
  Napi::Object obj = Napi::Object::New(env);
  switch (subscription_event_type) {
    case RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED: {
      obj.Set(
          "total_count",
          Napi::Number::New(env, data.requested_deadline_missed.total_count));
      obj.Set("total_count_change",
              Napi::Number::New(
                  env, data.requested_deadline_missed.total_count_change));
      break;
    }
    case RCL_SUBSCRIPTION_LIVELINESS_CHANGED: {
      obj.Set("alive_count",
              Napi::Number::New(env, data.liveliness_changed.alive_count));
      obj.Set("not_alive_count",
              Napi::Number::New(env, data.liveliness_changed.not_alive_count));
      obj.Set(
          "alive_count_change",
          Napi::Number::New(env, data.liveliness_changed.alive_count_change));
      obj.Set("not_alive_count_change",
              Napi::Number::New(
                  env, data.liveliness_changed.not_alive_count_change));
      break;
    }
    case RCL_SUBSCRIPTION_MESSAGE_LOST: {
      obj.Set("total_count",
              Napi::Number::New(env, data.message_lost.total_count));
      obj.Set("total_count_change",
              Napi::Number::New(env, data.message_lost.total_count_change));
      break;
    }
    case RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS: {
      obj.Set(
          "total_count",
          Napi::Number::New(env, data.requested_incompatible_qos.total_count));
      obj.Set("total_count_change",
              Napi::Number::New(
                  env, data.requested_incompatible_qos.total_count_change));
      obj.Set("last_policy_kind",
              Napi::Number::New(
                  env, data.requested_incompatible_qos.last_policy_kind));
      break;
    }
    case RCL_SUBSCRIPTION_INCOMPATIBLE_TYPE: {
      obj.Set("total_count",
              Napi::Number::New(env, data.incompatible_type.total_count));
      obj.Set(
          "total_count_change",
          Napi::Number::New(env, data.incompatible_type.total_count_change));
      break;
    }
    case RCL_SUBSCRIPTION_MATCHED: {
      obj.Set("total_count",
              Napi::Number::New(env, data.subscription_matched.total_count));
      obj.Set(
          "total_count_change",
          Napi::Number::New(env, data.subscription_matched.total_count_change));
      obj.Set("current_count",
              Napi::Number::New(env, data.subscription_matched.current_count));
      obj.Set("current_count_change",
              Napi::Number::New(
                  env, data.subscription_matched.current_count_change));
      break;
    }
    default:
      break;
  }
  return obj;
}

Napi::Value CreateJSObjectForPublisherEvent(
    Napi::Env env, rcl_publisher_event_type_t publisher_event_type,
    const event_callback_data_t& data) {
  Napi::Object obj = Napi::Object::New(env);
  switch (publisher_event_type) {
    case RCL_PUBLISHER_OFFERED_DEADLINE_MISSED: {
      obj.Set("total_count",
              Napi::Number::New(env, data.offered_deadline_missed.total_count));
      obj.Set("total_count_change",
              Napi::Number::New(
                  env, data.offered_deadline_missed.total_count_change));
      break;
    }
    case RCL_PUBLISHER_LIVELINESS_LOST: {
      obj.Set("total_count",
              Napi::Number::New(env, data.liveliness_lost.total_count));
      obj.Set("total_count_change",
              Napi::Number::New(env, data.liveliness_lost.total_count_change));
      break;
    }
    case RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS: {
      obj.Set(
          "total_count",
          Napi::Number::New(env, data.offered_incompatible_qos.total_count));
      obj.Set("total_count_change",
              Napi::Number::New(
                  env, data.offered_incompatible_qos.total_count_change));
      obj.Set("last_policy_kind",
              Napi::Number::New(
                  env, data.offered_incompatible_qos.last_policy_kind));
      break;
    }
    case RCL_PUBLISHER_INCOMPATIBLE_TYPE: {
      obj.Set("total_count",
              Napi::Number::New(env, data.incompatible_type.total_count));
      obj.Set(
          "total_count_change",
          Napi::Number::New(env, data.incompatible_type.total_count_change));
      break;
    }
    case RCL_PUBLISHER_MATCHED: {
      obj.Set("total_count",
              Napi::Number::New(env, data.publisher_matched.total_count));
      obj.Set(
          "total_count_change",
          Napi::Number::New(env, data.publisher_matched.total_count_change));
      obj.Set("current_count",
              Napi::Number::New(env, data.publisher_matched.current_count));
      obj.Set(
          "current_count_change",
          Napi::Number::New(env, data.publisher_matched.current_count_change));
      break;
    }
    default:
      break;
  }
  return obj;
}

}  // namespace

namespace rclnodejs {

Napi::Value CreateSubscriptionEventHandle(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());
  rcl_subscription_event_type_t event_type =
      static_cast<rcl_subscription_event_type_t>(
          info[1].As<Napi::Number>().Int32Value());

  rcl_event_t* event = CreateEventHandle();
  rcl_ret_t ret = rcl_subscription_event_init(event, subscription, event_type);

  if (ret != RCL_RET_OK) {
    Napi::Error::New(env, "failed to create subscription event")
        .ThrowAsJavaScriptException();
    rcl_reset_error();
    free(event);
    return env.Undefined();
  }

  auto js_obj = RclHandle::NewInstance(env, event, nullptr, [env](void* ptr) {
    rcl_event_t* event = reinterpret_cast<rcl_event_t*>(ptr);
    rcl_ret_t ret = rcl_event_fini(event);
    if (ret != RCL_RET_OK) {
      Napi::Error::New(env, rcl_get_error_string().str)
          .ThrowAsJavaScriptException();
      rcl_reset_error();
    }
    free(ptr);
  });
  return js_obj;
}

Napi::Value CreatePublisherEventHandle(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* publisher_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_publisher_t* publisher =
      reinterpret_cast<rcl_publisher_t*>(publisher_handle->ptr());
  rcl_publisher_event_type_t event_type =
      static_cast<rcl_publisher_event_type_t>(
          info[1].As<Napi::Number>().Int32Value());

  rcl_event_t* event = CreateEventHandle();
  rcl_ret_t ret = rcl_publisher_event_init(event, publisher, event_type);

  if (ret != RCL_RET_OK) {
    Napi::Error::New(env, "failed to create publisher event")
        .ThrowAsJavaScriptException();
    rcl_reset_error();
    free(event);
    return env.Undefined();
  }

  auto js_obj = RclHandle::NewInstance(env, event, nullptr, [env](void* ptr) {
    rcl_event_t* event = reinterpret_cast<rcl_event_t*>(ptr);
    rcl_ret_t ret = rcl_event_fini(event);
    if (ret != RCL_RET_OK) {
      Napi::Error::New(env, rcl_get_error_string().str)
          .ThrowAsJavaScriptException();
      rcl_reset_error();
    }
    free(ptr);
  });
  return js_obj;
}

Napi::Value TakeEvent(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* event_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_event_t* event = reinterpret_cast<rcl_event_t*>(event_handle->ptr());
  auto event_type = info[1].As<Napi::Object>();

  event_callback_data_t data;
  rcl_ret_t ret;
  if (event_type.Has("subscription_event_type")) {
    rcl_subscription_event_type_t subscription_event_type =
        static_cast<rcl_subscription_event_type_t>(
            event_type.Get("subscription_event_type")
                .As<Napi::Number>()
                .Int32Value());
    ret = rcl_take_event(event, &data);
    if (RCL_RET_OK == ret) {
      return CreateJSObjectForSubscriptionEvent(env, subscription_event_type,
                                                data);
    }
  } else if (event_type.Has("publisher_event_type")) {
    rcl_publisher_event_type_t publisher_event_type =
        static_cast<rcl_publisher_event_type_t>(
            event_type.Get("publisher_event_type")
                .As<Napi::Number>()
                .Int32Value());
    ret = rcl_take_event(event, &data);
    if (RCL_RET_OK == ret) {
      return CreateJSObjectForPublisherEvent(env, publisher_event_type, data);
    }
  }

  Napi::Error::New(env, "failed to take event").ThrowAsJavaScriptException();
  return env.Undefined();
}

Napi::Object InitEventHandleBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createSubscriptionEventHandle",
              Napi::Function::New(env, CreateSubscriptionEventHandle));
  exports.Set("createPublisherEventHandle",
              Napi::Function::New(env, CreatePublisherEventHandle));
  exports.Set("takeEvent", Napi::Function::New(env, TakeEvent));
  return exports;
}

}  // namespace rclnodejs
