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

#include <nan.h>

#include "macros.hpp"
#include "rcl_action_bindings.hpp"
#include "rcl_bindings.hpp"
#include "rcl_handle.hpp"
#include "rcutils/logging.h"
#include "rcutils/macros.h"
#include "shadow_node.hpp"

void InitModule(v8::Local<v8::Object> exports) {
  v8::Local<v8::Context> context = exports->GetIsolate()->GetCurrentContext();

  for (uint32_t i = 0; i < rclnodejs::binding_methods.size(); i++) {
    Nan::Set(
        exports, Nan::New(rclnodejs::binding_methods[i].name).ToLocalChecked(),
        Nan::New<v8::FunctionTemplate>(rclnodejs::binding_methods[i].function)
            ->GetFunction(context)
            .ToLocalChecked());
  }

  for (uint32_t i = 0; i < rclnodejs::action_binding_methods.size(); i++) {
    Nan::Set(
        exports,
        Nan::New(rclnodejs::action_binding_methods[i].name).ToLocalChecked(),
        Nan::New<v8::FunctionTemplate>(
            rclnodejs::action_binding_methods[i].function)
            ->GetFunction(context)
            .ToLocalChecked());
  }

  rclnodejs::ShadowNode::Init(exports);
  rclnodejs::RclHandle::Init(exports);

#ifdef DEBUG_ON
  int result = rcutils_logging_set_logger_level(PACKAGE_NAME,
                                                RCUTILS_LOG_SEVERITY_DEBUG);
  RCUTILS_UNUSED(result);
#endif
}

NODE_MODULE(rclnodejs, InitModule);
