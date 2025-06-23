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

#include "rcl_client_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

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

    auto js_obj = RclHandle::NewInstance(
        env, client, node_handle, [node, env](void* ptr) {
          rcl_client_t* client = reinterpret_cast<rcl_client_t*>(ptr);
          rcl_ret_t ret = rcl_client_fini(client, node);
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

Napi::Value GetClientServiceName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* service_name = rcl_client_get_service_name(client);
  return Napi::String::New(env, service_name);
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

#if ROS_VERSION > 2205  // 2205 == Humble
Napi::Value ConfigureClientIntrospection(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* client_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_client_t* client = reinterpret_cast<rcl_client_t*>(client_handle->ptr());
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

    THROW_ERROR_IF_NOT_EQUAL(rcl_client_configure_service_introspection(
                                 client, node, clock, ts, publisher_ops, state),
                             RCL_RET_OK, rcl_get_error_string().str);
  } else {
    Napi::Error::New(env, GetErrorMessageAndClear())
        .ThrowAsJavaScriptException();
  }
  return env.Undefined();
}
#endif

Napi::Object InitClientBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createClient", Napi::Function::New(env, CreateClient));
  exports.Set("sendRequest", Napi::Function::New(env, SendRequest));
  exports.Set("rclTakeResponse", Napi::Function::New(env, RclTakeResponse));
  exports.Set("getClientServiceName",
              Napi::Function::New(env, GetClientServiceName));
  exports.Set("serviceServerIsAvailable",
              Napi::Function::New(env, ServiceServerIsAvailable));
#if ROS_VERSION > 2205  // 2205 == Humble
  exports.Set("configureClientIntrospection",
              Napi::Function::New(env, ConfigureClientIntrospection));
#endif
  return exports;
}

}  // namespace rclnodejs
