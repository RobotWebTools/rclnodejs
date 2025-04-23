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

#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

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
  exports.Set("getNodeNames", Napi::Function::New(env, GetNodeNames));
  exports.Set("countPublishers", Napi::Function::New(env, CountPublishers));
  exports.Set("countSubscribers", Napi::Function::New(env, CountSubscribers));
  exports.Set("serviceServerIsAvailable",
              Napi::Function::New(env, ServiceServerIsAvailable));
  return exports;
}

}  // namespace rclnodejs
