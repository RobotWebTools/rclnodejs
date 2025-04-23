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

#include "rcl_names_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/rcl.h>
#include <rcl/validate_topic_name.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>

#include <string>

#include "macros.h"
#include "rcl_utilities.h"

namespace rclnodejs {

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

Napi::Object InitNamesBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("validateFullTopicName",
              Napi::Function::New(env, ValidateFullTopicName));
  exports.Set("validateNodeName", Napi::Function::New(env, ValidateNodeName));
  exports.Set("validateTopicName", Napi::Function::New(env, ValidateTopicName));
  exports.Set("validateNamespace", Napi::Function::New(env, ValidateNamespace));
  exports.Set("expandTopicName", Napi::Function::New(env, ExpandTopicName));
  return exports;
}

}  // namespace rclnodejs
