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

namespace rclnodejs {

typedef void (*JsCFuntcion)(const Nan::FunctionCallbackInfo<v8::Value>&);

typedef struct {
  const char* name;
  JsCFuntcion function;
} BindingMethod;

uint32_t GetBindingMethodsCount(BindingMethod* methods);

extern BindingMethod binding_methods[];

}  // namespace rclnodejs

#endif
