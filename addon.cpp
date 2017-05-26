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

#include "shadow_node.hpp"
#include "rcl_bindings.hpp"
#include "rcl_handle.hpp"

void InitModule(v8::Local<v8::Object> exports) {
  for (uint32_t i = 0; i < rclnodejs::GetBindingMethodsCount(
        rclnodejs::binding_methods); i++) {
    exports->Set(Nan::New(rclnodejs::binding_methods[i].name).ToLocalChecked(),
                 Nan::New<v8::FunctionTemplate>(
                     rclnodejs::binding_methods[i].function)->GetFunction());
  }

  rclnodejs::ShadowNode::Init(exports);
  rclnodejs::RclHandle::Init(exports);
}

NODE_MODULE(rclnodejs, InitModule);
