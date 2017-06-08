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

#ifndef RCLNODEJS_RCL_BINDINGS_HPP_
#define RCLNODEJS_RCL_BINDINGS_HPP_

#include <nan.h>

// This macro will check the return code of the func_call
//  It will throw an exception to JavaScript if there is any thing went wrong
#define RCLN_CHECK_AND_THROW(func_call) \
  { \
    auto rclnodejs_ret_code = (func_call); \
    if (rclnodejs_ret_code != RCL_RET_OK) { \
      Nan::ThrowError(rcl_get_error_string_safe()); \
    } \
  }

namespace rclnodejs {

typedef void (*JsCFuntcion)(const Nan::FunctionCallbackInfo<v8::Value>&);

typedef struct {
  const char* name;
  JsCFuntcion function;
} BindingMethod;

uint32_t GetBindingMethodsCount(BindingMethod* methods);

extern BindingMethod binding_methods[];

// Publisher methods
NAN_METHOD(CreatePublisher);
NAN_METHOD(rcl_publish_std_string_message); // Temp method
NAN_METHOD(PublishMessage);

}  // namespace rclnodejs

#endif
