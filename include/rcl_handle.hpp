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

#ifndef RCLNODEJS_RCL_HANDLE_HPP_
#define RCLNODEJS_RCL_HANDLE_HPP_

#include <nan.h>

namespace rclnodejs {

class RclHandle : public Nan::ObjectWrap {
 public:
  static void Init(v8::Local<v8::Object> exports);
  static v8::Local<v8::Object> NewInstance(void* handle);

  void* GetPtr() { return handle_; }
  void SetPtr(void* handle) { handle_ = handle; }

 private:
  RclHandle();
  ~RclHandle();

  static Nan::Persistent<v8::Function> constructor;
  static void New(const Nan::FunctionCallbackInfo<v8::Value>& info);
  void* handle_;
};

}  // namespace rclnodejs

#endif
