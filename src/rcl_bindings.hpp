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
#include <rcl/graph.h>
#include <rcl/rcl.h>

#include <memory>
#include <string>
#include <vector>

namespace rclnodejs {

typedef void (*JsCFuntcion)(const Nan::FunctionCallbackInfo<v8::Value>&);

typedef struct {
  const char* name;
  JsCFuntcion function;
} BindingMethod;

extern rcl_guard_condition_t* g_sigint_gc;

void ExtractNamesAndTypes(rcl_names_and_types_t names_and_types,
                          v8::Local<v8::Array>* result_list);

std::unique_ptr<rmw_qos_profile_t> GetQoSProfile(v8::Local<v8::Value> qos);

extern std::vector<BindingMethod> binding_methods;

}  // namespace rclnodejs

#endif
