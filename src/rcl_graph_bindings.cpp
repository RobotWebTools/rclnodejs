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

#include "rcl_graph_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/rcl.h>

#include <rcpputils/scope_exit.hpp>
// NOLINTNEXTLINE
#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

typedef rcl_ret_t (*rcl_get_info_by_topic_func_t)(
    const rcl_node_t* node, rcutils_allocator_t* allocator,
    const char* topic_name, bool no_mangle,
    rcl_topic_endpoint_info_array_t* info_array);

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
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_get_client_names_and_types_by_node(
                               node, &allocator, node_name.c_str(),
                               node_namespace.c_str(), &client_names_and_types),
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

Napi::Value GetInfoByTopic(Napi::Env env, rcl_node_t* node,
                           const char* topic_name, bool no_mangle,
                           const char* type,
                           rcl_get_info_by_topic_func_t rcl_get_info_by_topic) {
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_topic_endpoint_info_array_t info_array =
      rcl_get_zero_initialized_topic_endpoint_info_array();

  RCPPUTILS_SCOPE_EXIT({
    rcl_ret_t fini_ret =
        rcl_topic_endpoint_info_array_fini(&info_array, &allocator);
    if (RCL_RET_OK != fini_ret) {
      Napi::Error::New(env, rcl_get_error_string().str)
          .ThrowAsJavaScriptException();
      rcl_reset_error();
    }
  });

  rcl_ret_t ret = rcl_get_info_by_topic(node, &allocator, topic_name, no_mangle,
                                        &info_array);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_UNSUPPORTED == ret) {
      Napi::Error::New(
          env, std::string("Failed to get information by topic for ") + type +
                   ": function not supported by RMW_IMPLEMENTATION")
          .ThrowAsJavaScriptException();
      return env.Undefined();
    }
    Napi::Error::New(
        env, std::string("Failed to get information by topic for ") + type)
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  return ConvertToJSTopicEndpointInfoList(env, &info_array);
}

Napi::Value GetPublishersInfoByTopic(const Napi::CallbackInfo& info) {
  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();
  bool no_mangle = info[2].As<Napi::Boolean>();

  return GetInfoByTopic(info.Env(), node, topic_name.c_str(), no_mangle,
                        "publishers", rcl_get_publishers_info_by_topic);
}

Napi::Value GetSubscriptionsInfoByTopic(const Napi::CallbackInfo& info) {
  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();
  bool no_mangle = info[2].As<Napi::Boolean>();

  return GetInfoByTopic(info.Env(), node, topic_name.c_str(), no_mangle,
                        "subscriptions", rcl_get_subscriptions_info_by_topic);
}

Napi::Object InitGraphBindings(Napi::Env env, Napi::Object exports) {
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
  exports.Set("getPublishersInfoByTopic",
              Napi::Function::New(env, GetPublishersInfoByTopic));
  exports.Set("getSubscriptionsInfoByTopic",
              Napi::Function::New(env, GetSubscriptionsInfoByTopic));
  return exports;
}

}  // namespace rclnodejs
