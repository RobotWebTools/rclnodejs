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
#include <rcl/subscription.h>
#include <rmw/types.h>

#include <cstdio>
#include <memory>
#include <string>

#include <rcpputils/scope_exit.hpp>

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
    std::string error_string = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return Napi::Boolean::New(env, false);
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    return Napi::Boolean::New(env, true);
  }

  return env.Undefined();
}

Napi::Value RclTakeWithInfo(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());
  void* msg_taken = info[1].As<Napi::Buffer<char>>().Data();

  rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
  rcl_ret_t ret = rcl_take(subscription, msg_taken, &message_info, nullptr);

  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    std::string error_string = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    return env.Undefined();
  }

  // Build JS object with message info fields
  Napi::Object js_info = Napi::Object::New(env);
  js_info.Set("source_timestamp",
              Napi::BigInt::New(env, message_info.source_timestamp));
  js_info.Set("received_timestamp",
              Napi::BigInt::New(env, message_info.received_timestamp));
  js_info.Set(
      "publication_sequence_number",
      Napi::BigInt::New(
          env, static_cast<int64_t>(message_info.publication_sequence_number)));
  js_info.Set(
      "reception_sequence_number",
      Napi::BigInt::New(
          env, static_cast<int64_t>(message_info.reception_sequence_number)));

  // Publisher GID as Buffer
  auto gid_buf =
      Napi::Buffer<uint8_t>::Copy(env, message_info.publisher_gid.data,
                                  sizeof(message_info.publisher_gid.data));
  js_info.Set("publisher_gid", gid_buf);

  return js_info;
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
            argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char)));
            snprintf(argv[i], len, "%s", arg.c_str());
          }
        }
      }

      rcl_ret_t ret = rcl_subscription_options_set_content_filter_options(
          expression.c_str(), argc, (const char**)argv, &subscription_ops);

      if (argc) {
        for (int i = 0; i < argc; i++) {
          free(argv[i]);
        }
        free(argv);
      }

      if (ret != RCL_RET_OK) {
        std::string error_string = rcl_get_error_string().str;
        rcl_reset_error();
        free(subscription);
        Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
        return env.Undefined();
      }
    }
  }

  const rosidl_message_type_support_t* ts =
      GetMessageTypeSupport(package_name, message_sub_folder, message_name);

  if (ts) {
    rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic.c_str(),
                                          &subscription_ops);
    if (ret != RCL_RET_OK) {
      std::string error_msg = rcl_get_error_string().str;
      rcl_reset_error();
      free(subscription);
      Napi::Error::New(env, error_msg).ThrowAsJavaScriptException();
      return env.Undefined();
    }

    auto js_obj = RclHandle::NewInstance(
        env, subscription, node_handle, [node, env](void* ptr) {
          rcl_subscription_t* subscription =
              reinterpret_cast<rcl_subscription_t*>(ptr);
          rcl_ret_t ret = rcl_subscription_fini(subscription, node);
          free(ptr);
          if (ret != RCL_RET_OK) {
            std::string error_msg = rcl_get_error_string().str;
            rcl_reset_error();
            Napi::Error::New(env, error_msg).ThrowAsJavaScriptException();
          }
        });

    return js_obj;
  } else {
    std::string error_msg = GetErrorMessageAndClear();
    free(subscription);
    Napi::Error::New(env, error_msg).ThrowAsJavaScriptException();
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

  RCPPUTILS_SCOPE_EXIT({
    rcl_ret_t fini_ret = rmw_serialized_message_fini(&msg);
    if (fini_ret != RCL_RET_OK) {
      rcl_reset_error();
    }
  });

  Napi::Buffer<char> buffer = Napi::Buffer<char>::Copy(
      env, reinterpret_cast<char*>(msg.buffer), msg.buffer_length);

  return buffer;
}

Napi::Value GetSubscriptionTopic(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_subscription_t* subscription = reinterpret_cast<rcl_subscription_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* topic = rcl_subscription_get_topic_name(subscription);
  return Napi::String::New(env, topic);
}

#if ROS_VERSION > 2505  // Rolling only
Napi::Value IsContentFilterSupported(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  bool is_supported = rcl_subscription_is_cft_supported(subscription);
  return Napi::Boolean::New(env, is_supported);
}
#endif

Napi::Value HasContentFilter(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  bool is_valid = rcl_subscription_is_cft_enabled(subscription);
  return Napi::Boolean::New(env, is_valid);
}

Napi::Value SetContentFilter(const Napi::CallbackInfo& info) {
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
        argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char)));
        snprintf(argv[i], len, "%s", arg.c_str());
      }
    }
  }

  // create ctf options
  rcl_subscription_content_filter_options_t options =
      rcl_get_zero_initialized_subscription_content_filter_options();

  rcl_ret_t ret = rcl_subscription_content_filter_options_set(
      subscription, expression.c_str(), argc, (const char**)argv, &options);

  if (ret != RCL_RET_OK) {
    if (argc) {
      for (int i = 0; i < argc; i++) {
        free(argv[i]);
      }
      free(argv);
    }
    std::string error_string = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  ret = rcl_subscription_set_content_filter(subscription, &options);

  if (argc) {
    for (int i = 0; i < argc; i++) {
      free(argv[i]);
    }
    free(argv);
  }

  std::string error_string = "";
  if (ret != RCL_RET_OK) {
    error_string = rcl_get_error_string().str;
    rcl_reset_error();
  }

  rcl_ret_t fini_ret =
      rcl_subscription_content_filter_options_fini(subscription, &options);

  if (ret != RCL_RET_OK) {
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  if (fini_ret != RCL_RET_OK) {
    error_string = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  return Napi::Boolean::New(env, true);
}

Napi::Value ClearContentFilter(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  // create ctf options
  rcl_subscription_content_filter_options_t options =
      rcl_get_zero_initialized_subscription_content_filter_options();

  rcl_ret_t ret = rcl_subscription_content_filter_options_init(
      subscription, "", 0, (const char**)nullptr, &options);

  if (ret != RCL_RET_OK) {
    std::string error_string = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  ret = rcl_subscription_set_content_filter(subscription, &options);
  rcl_ret_t fini_ret =
      rcl_subscription_content_filter_options_fini(subscription, &options);

  if (ret != RCL_RET_OK) {
    std::string error_string = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  if (fini_ret != RCL_RET_OK) {
    std::string error_string = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_string).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  return Napi::Boolean::New(env, true);
}

Napi::Value GetContentFilter(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* subscription_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->ptr());

  rcl_subscription_content_filter_options_t options =
      rcl_get_zero_initialized_subscription_content_filter_options();

  rcl_ret_t ret = rcl_subscription_get_content_filter(subscription, &options);
  if (ret != RCL_RET_OK) {
    std::string error_msg = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_msg).ThrowAsJavaScriptException();
    return env.Undefined();
  }

  RCPPUTILS_SCOPE_EXIT({
    rcl_ret_t fini_ret =
        rcl_subscription_content_filter_options_fini(subscription, &options);
    if (fini_ret != RCL_RET_OK) {
      rcl_reset_error();
    }
  });

  // Create result object
  Napi::Object result = Napi::Object::New(env);
  result.Set(
      "expression",
      Napi::String::New(
          env,
          options.rmw_subscription_content_filter_options.filter_expression));

  size_t param_count = options.rmw_subscription_content_filter_options
                           .expression_parameters.size;
  Napi::Array parameters = Napi::Array::New(env, param_count);
  for (size_t i = 0; i < param_count; ++i) {
    parameters[i] =
        Napi::String::New(env, options.rmw_subscription_content_filter_options
                                   .expression_parameters.data[i]);
  }
  result.Set("parameters", parameters);

  return result;
}

Napi::Value GetPublisherCount(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_subscription_t* subscription = reinterpret_cast<rcl_subscription_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  size_t count = 0;
  rcl_ret_t ret = rcl_subscription_get_publisher_count(subscription, &count);
  if (ret != RCL_RET_OK) {
    std::string error_msg = rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env, error_msg).ThrowAsJavaScriptException();
  }

  return Napi::Number::New(env, count);
}

Napi::Object InitSubscriptionBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("rclTake", Napi::Function::New(env, RclTake));
  exports.Set("rclTakeWithInfo", Napi::Function::New(env, RclTakeWithInfo));
  exports.Set("createSubscription",
              Napi::Function::New(env, CreateSubscription));
  exports.Set("rclTakeRaw", Napi::Function::New(env, RclTakeRaw));
  exports.Set("getSubscriptionTopic",
              Napi::Function::New(env, GetSubscriptionTopic));
#if ROS_VERSION > 2505  // Rolling only
  exports.Set("isContentFilterSupported",
              Napi::Function::New(env, IsContentFilterSupported));
#endif
  exports.Set("hasContentFilter", Napi::Function::New(env, HasContentFilter));
  exports.Set("setContentFilter", Napi::Function::New(env, SetContentFilter));
  exports.Set("getContentFilter", Napi::Function::New(env, GetContentFilter));
  exports.Set("clearContentFilter",
              Napi::Function::New(env, ClearContentFilter));
  exports.Set("getPublisherCount", Napi::Function::New(env, GetPublisherCount));
  return exports;
}

}  // namespace rclnodejs
