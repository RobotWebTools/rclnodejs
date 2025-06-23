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

#include "rcl_subscription_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <cstdio>
#include <memory>
#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

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
        env, subscription, node_handle, [node, env](void* ptr) {
          rcl_subscription_t* subscription =
              reinterpret_cast<rcl_subscription_t*>(ptr);
          rcl_ret_t ret = rcl_subscription_fini(subscription, node);
          free(ptr);
          THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                             rcl_get_error_string().str);
        });

    return js_obj;
  } else {
    Napi::Error::New(env, GetErrorMessageAndClear())
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }
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

Napi::Value GetSubscriptionTopic(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_subscription_t* subscription = reinterpret_cast<rcl_subscription_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* topic = rcl_subscription_get_topic_name(subscription);
  return Napi::String::New(env, topic);
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

Napi::Value GetPublisherCount(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_subscription_t* subscription = reinterpret_cast<rcl_subscription_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_subscription_get_publisher_count(subscription, &count), RCL_RET_OK,
      rcl_get_error_string().str);

  return Napi::Number::New(env, count);
}

Napi::Object InitSubscriptionBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("rclTake", Napi::Function::New(env, RclTake));
  exports.Set("createSubscription",
              Napi::Function::New(env, CreateSubscription));
  exports.Set("rclTakeRaw", Napi::Function::New(env, RclTakeRaw));
  exports.Set("getSubscriptionTopic",
              Napi::Function::New(env, GetSubscriptionTopic));
  exports.Set("hasContentFilter", Napi::Function::New(env, HasContentFilter));
  exports.Set("setContentFilter", Napi::Function::New(env, SetContentFilter));
  exports.Set("clearContentFilter",
              Napi::Function::New(env, ClearContentFilter));
  exports.Set("getPublisherCount", Napi::Function::New(env, GetPublisherCount));
  return exports;
}

}  // namespace rclnodejs
