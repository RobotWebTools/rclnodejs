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

#include "rcl_utilities.h"

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rmw/topic_endpoint_info.h>
#include <uv.h>

#include <cstdio>
#include <memory>
#include <string>

namespace {

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

Napi::Value ConvertRMWTimeToDuration(Napi::Env env,
                                     const rmw_time_t* duration) {
  Napi::Object obj = Napi::Object::New(env);
  obj.Set("seconds", Napi::BigInt::New(env, duration->sec));
  obj.Set("nanoseconds", Napi::Number::New(env, duration->nsec));
  return obj;
}

#if ROS_VERSION > 2205  // 2205 == Humble
Napi::Value ConvertToHashObject(Napi::Env env,
                                const rosidl_type_hash_t* type_hash) {
  Napi::Object obj = Napi::Object::New(env);
  obj.Set("version", Napi::Number::New(env, type_hash->version));
  obj.Set("value", Napi::Buffer<char>::Copy(
                       env, reinterpret_cast<const char*>(type_hash->value),
                       ROSIDL_TYPE_HASH_SIZE));
  return obj;
}
#endif

Napi::Value ConvertToJSTopicEndpoint(
    Napi::Env env, const rmw_topic_endpoint_info_t* topic_endpoint_info) {
  Napi::Array endpoint_gid = Napi::Array::New(env, RMW_GID_STORAGE_SIZE);
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++) {
    endpoint_gid.Set(
        i, Napi::Number::New(env, topic_endpoint_info->endpoint_gid[i]));
  }

  Napi::Object endpoint = Napi::Object::New(env);
  endpoint.Set("node_name",
               Napi::String::New(env, topic_endpoint_info->node_name));
  endpoint.Set("node_namespace",
               Napi::String::New(env, topic_endpoint_info->node_namespace));
  endpoint.Set("topic_type",
               Napi::String::New(env, topic_endpoint_info->topic_type));
#if ROS_VERSION > 2205  // 2205 == Humble
  endpoint.Set("topic_type_hash",
               ConvertToHashObject(env, &topic_endpoint_info->topic_type_hash));
#endif
  endpoint.Set("endpoint_type",
               Napi::Number::New(
                   env, static_cast<int>(topic_endpoint_info->endpoint_type)));
  endpoint.Set("endpoint_gid", endpoint_gid);
  endpoint.Set("qos_profile",
               rclnodejs::ConvertToQoS(env, &topic_endpoint_info->qos_profile));
  return endpoint;
}

uv_lib_t g_lib;
Napi::Env g_env = nullptr;

}  // namespace

namespace rclnodejs {

typedef const rosidl_message_type_support_t* (*GetMessageTypeSupportFunction)();
typedef const rosidl_service_type_support_t* (*GetServiceTypeSupportFunction)();
typedef const rosidl_action_type_support_t* (*GetActionTypeSupportFunction)();

#if defined(OS_MACOS)
const char* lib_prefix = "lib";
const char* lib_ext = ".dylib";
#elif defined(OS_LINUX)
const char* lib_prefix = "lib";
const char* lib_ext = ".so";
#elif defined(OS_WINDOWS)
const char* lib_prefix = "";
const char* lib_ext = ".dll";
#endif

void* GetTypeSupportFunctionByInterfaceSymbolName(
    const std::string& symbol_name, const std::string& lib_name) {
  // If the dlopen fails for any reason, it will return nullptr.
  // You can use GetErrorMessageAndClear() to get error diagnostic.
  void* ptr = nullptr;
  if (uv_dlopen(lib_name.c_str(), &g_lib) == 0 &&
      uv_dlsym(&g_lib, symbol_name.c_str(), reinterpret_cast<void**>(&ptr)) ==
          0) {
    return ptr;
  }
  return nullptr;
}

const rosidl_message_type_support_t* GetMessageTypeSupport(
    const std::string& package_name, const std::string& sub_folder,
    const std::string& msg_name) {
  void* function = GetTypeSupportFunctionByInterfaceSymbolName(
      "rosidl_typesupport_c__get_message_type_support_handle__" + package_name +
          "__" + sub_folder + "__" + msg_name,
      lib_prefix + package_name + "__rosidl_typesupport_c" + lib_ext);
  if (function)
    return reinterpret_cast<GetMessageTypeSupportFunction>(function)();
  else
    return nullptr;
}

const rosidl_service_type_support_t* GetServiceTypeSupport(
    const std::string& package_name, const std::string& service_name) {
  void* function = GetTypeSupportFunctionByInterfaceSymbolName(
      "rosidl_typesupport_c__get_service_type_support_handle__" + package_name +
          "__srv__" + service_name,
      lib_prefix + package_name + "__rosidl_typesupport_c" + lib_ext);
  if (function)
    return reinterpret_cast<GetServiceTypeSupportFunction>(function)();
  else
    return nullptr;
}

const rosidl_action_type_support_t* GetActionTypeSupport(
    const std::string& package_name, const std::string& action_name) {
  void* function = GetTypeSupportFunctionByInterfaceSymbolName(
      "rosidl_typesupport_c__get_action_type_support_handle__" + package_name +
          "__action__" + action_name,
      lib_prefix + package_name + "__rosidl_typesupport_c" + lib_ext);
  if (function)
    return reinterpret_cast<GetActionTypeSupportFunction>(function)();
  else
    return nullptr;
}

std::string GetErrorMessageAndClear() {
  return std::string(uv_dlerror(&g_lib));
}

Napi::Env& GetEnv() { return g_env; }

void StoreEnv(Napi::Env current_env) { g_env = current_env; }

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

Napi::Value ConvertToQoS(Napi::Env env, const rmw_qos_profile_t* qos_profile) {
  Napi::Object qos = Napi::Object::New(env);
  qos.Set("depth", Napi::Number::New(env, qos_profile->depth));
  qos.Set("history", Napi::Number::New(env, qos_profile->history));
  qos.Set("reliability", Napi::Number::New(env, qos_profile->reliability));
  qos.Set("durability", Napi::Number::New(env, qos_profile->durability));
  qos.Set("lifespan", ConvertRMWTimeToDuration(env, &qos_profile->lifespan));
  qos.Set("deadline", ConvertRMWTimeToDuration(env, &qos_profile->deadline));
  qos.Set("liveliness", Napi::Number::New(env, qos_profile->liveliness));
  qos.Set(
      "liveliness_lease_duration",
      ConvertRMWTimeToDuration(env, &qos_profile->liveliness_lease_duration));
  qos.Set(
      "avoid_ros_namespace_conventions",
      Napi::Boolean::New(env, qos_profile->avoid_ros_namespace_conventions));
  return qos;
}

Napi::Array ConvertToJSTopicEndpointInfoList(
    Napi::Env env, const rmw_topic_endpoint_info_array_t* info_array) {
  Napi::Array list = Napi::Array::New(env, info_array->size);
  for (size_t i = 0; i < info_array->size; ++i) {
    rmw_topic_endpoint_info_t topic_endpoint_info = info_array->info_array[i];
    list.Set(i, ConvertToJSTopicEndpoint(env, &topic_endpoint_info));
  }
  return list;
}

char** AbstractArgsFromNapiArray(const Napi::Array& jsArgv) {
  size_t argc = jsArgv.Length();
  char** argv = nullptr;

  if (argc > 0) {
    argv = reinterpret_cast<char**>(malloc(argc * sizeof(char*)));
    for (size_t i = 0; i < argc; i++) {
      std::string arg = jsArgv.Get(i).As<Napi::String>().Utf8Value();
      int len = arg.length() + 1;
      argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char)));
      snprintf(argv[i], len, "%s", arg.c_str());
    }
  }
  return argv;
}

void FreeArgs(char** argv, size_t argc) {
  if (argv) {
    for (size_t i = 0; i < argc; i++) {
      free(argv[i]);
    }
    free(argv);
  }
}

bool HasUnparsedROSArgs(const rcl_arguments_t& rcl_args) {
  int unparsed_ros_args_count = rcl_arguments_get_count_unparsed_ros(&rcl_args);
  return unparsed_ros_args_count != 0;
}

}  // namespace rclnodejs
