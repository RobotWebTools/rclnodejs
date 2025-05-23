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

#include "rcl_type_description_service_bindings.h"

#include <napi.h>
#include <rcl/rcl.h>
#include <rmw/types.h>

#include "rcl_handle.h"

namespace rclnodejs {

Napi::Value InitTypeDescriptionService(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  rcl_service_t* service =
      reinterpret_cast<rcl_service_t*>(malloc(sizeof(rcl_service_t)));
  *service = rcl_get_zero_initialized_service();
  rcl_ret_t ret = rcl_node_type_description_service_init(service, node);
  if (RCL_RET_OK != ret) {
    Napi::Error::New(env, "Failed to initialize type description service")
        .ThrowAsJavaScriptException();
  }

  auto service_handle =
      RclHandle::NewInstance(env, service, node_handle, [node, env](void* ptr) {
        rcl_service_t* service = reinterpret_cast<rcl_service_t*>(ptr);
        rcl_ret_t ret = rcl_service_fini(service, node);
        if (RCL_RET_OK != ret) {
          Napi::Error::New(env, "Failed to destroy type description service")
              .ThrowAsJavaScriptException();
        }
        free(ptr);
      });
  return service_handle;
}

Napi::Value HandleRequest(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  void* request = info[1].As<Napi::Buffer<char>>().Data();
  void* taken_response = info[2].As<Napi::Buffer<char>>().Data();

  rmw_request_id_t header;
  rcl_node_type_description_service_handle_request(
      node, &header,
      static_cast<
          type_description_interfaces__srv__GetTypeDescription_Request*>(
          request),
      static_cast<
          type_description_interfaces__srv__GetTypeDescription_Response*>(
          taken_response));
  return env.Undefined();
}

Napi::Object InitTypeDescriptionServiceBindings(Napi::Env env,
                                                Napi::Object exports) {
  exports.Set("handleRequest", Napi::Function::New(env, HandleRequest));
  exports.Set("initTypeDescriptionService",
              Napi::Function::New(env, InitTypeDescriptionService));

  return exports;
}

}  // namespace rclnodejs
