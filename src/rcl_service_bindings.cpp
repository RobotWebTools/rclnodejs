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

#include "rcl_service_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#if ROS_VERSION > 2205
#include <rcl/service_introspection.h>
#endif

#include <memory>
#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value CreateService(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  std::string service_name = info[1].As<Napi::String>().Utf8Value();
  std::string interface_name = info[2].As<Napi::String>().Utf8Value();
  std::string package_name = info[3].As<Napi::String>().Utf8Value();

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
    auto js_obj = RclHandle::NewInstance(
        env, service, node_handle, [node, env](void* ptr) {
          rcl_service_t* service = reinterpret_cast<rcl_service_t*>(ptr);
          rcl_ret_t ret = rcl_service_fini(service, node);
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

Napi::Value RclTakeRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  rmw_request_id_t* header =
      reinterpret_cast<rmw_request_id_t*>(malloc(sizeof(rmw_request_id_t)));

  void* taken_request = info[2].As<Napi::Buffer<char>>().Data();
  rcl_ret_t ret = rcl_take_request(service, header, taken_request);
  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    auto js_obj = RclHandle::NewInstance(env, header, nullptr,
                                         [](void* ptr) { free(ptr); });
    return js_obj;
  }

  return env.Undefined();
}

Napi::Value SendResponse(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  void* buffer = info[1].As<Napi::Buffer<char>>().Data();

  rmw_request_id_t* header = reinterpret_cast<rmw_request_id_t*>(
      RclHandle::Unwrap(info[2].As<Napi::Object>())->ptr());

  THROW_ERROR_IF_NOT_EQUAL(rcl_send_response(service, header, buffer),
                           RCL_RET_OK, rcl_get_error_string().str);

  return env.Undefined();
}

Napi::Value GetServiceServiceName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const char* service_name = rcl_service_get_service_name(service);
  return Napi::String::New(env, service_name);
}

#if ROS_VERSION > 2205  // 2205 == Humble
Napi::Value ConfigureServiceIntrospection(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* service_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_service_t* service =
      reinterpret_cast<rcl_service_t*>(service_handle->ptr());
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

    THROW_ERROR_IF_NOT_EQUAL(
        rcl_service_configure_service_introspection(service, node, clock, ts,
                                                    publisher_ops, state),
        RCL_RET_OK, rcl_get_error_string().str);
  } else {
    Napi::Error::New(env, GetErrorMessageAndClear())
        .ThrowAsJavaScriptException();
  }
  return env.Undefined();
}
#endif

Napi::Value GetOptions(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_service_t* service = reinterpret_cast<rcl_service_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());

  const rcl_service_options_t* options = rcl_service_get_options(service);
  auto qos_profile = ConvertToQoS(env, &options->qos);

  return qos_profile;
}

Napi::Object InitServiceBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createService", Napi::Function::New(env, CreateService));
  exports.Set("rclTakeRequest", Napi::Function::New(env, RclTakeRequest));
  exports.Set("sendResponse", Napi::Function::New(env, SendResponse));
  exports.Set("getServiceServiceName",
              Napi::Function::New(env, GetServiceServiceName));
#if ROS_VERSION > 2205  // 2205 == Humble
  exports.Set("configureServiceIntrospection",
              Napi::Function::New(env, ConfigureServiceIntrospection));
#endif
  exports.Set("getOptions", Napi::Function::New(env, GetOptions));
  return exports;
}

}  // namespace rclnodejs
