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

#ifndef SRC_RCL_BINDINGS_HPP_
#define SRC_RCL_BINDINGS_HPP_

#include <napi.h>
#include <rcl/graph.h>
#include <rcl/rcl.h>

#include <memory>
#include <string>
#include <vector>

namespace rclnodejs {

typedef Napi::Value (*JsCFunction)(const Napi::CallbackInfo& info);

typedef struct {
  const char* name;
  JsCFunction function;
} BindingMethod;

extern rcl_guard_condition_t* g_sigint_gc;

void ExtractNamesAndTypes(rcl_names_and_types_t names_and_types,
                          Napi::Array* result_list);

std::unique_ptr<rmw_qos_profile_t> GetQoSProfile(Napi::Value qos);

Napi::Object InitBindings(Napi::Env env, Napi::Object exports);

}  // namespace rclnodejs

#endif  // SRC_RCL_BINDINGS_HPP_
