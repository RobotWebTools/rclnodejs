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
#include <string>

// This macro will check the return code of the func_call
// It will throw an exception to JavaScript if there is any thing went wrong
#define RCLN_CHECK_AND_THROW(func_call, expected) \
  { \
    if (func_call != (expected)) { \
      Nan::ThrowError(rcl_get_error_string_safe()); \
      info.GetReturnValue().Set(Nan::Undefined()); \
      return; \
    } \
  }

#define RCLN_THROW_EQUAL(value, constant, message) \
  { \
    if ((value) == (constant)) { \
      Nan::ThrowError((message)); \
      info.GetReturnValue().Set(Nan::Undefined()); \
      return; \
    } \
  }

#define RCLN_THROW_NOT_EQUAL(value, constant, message) \
  { \
    if ((value) != (constant)) { \
      Nan::ThrowError((message)); \
      info.GetReturnValue().Set(Nan::Undefined()); \
      return; \
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

inline std::string RCLN_GET_MSG_TYPE_SUPPORT(const std::string& packageName,
    const std::string& messageSubFolder,
    const std::string& messageName) {
  std::string name("rosidl_typesupport_c__get_message_type_support_handle__");
  name += packageName;
  name += "__";
  name += messageSubFolder;
  name += "__";
  name += messageName;
  return name;
}

}  // namespace rclnodejs

#endif
