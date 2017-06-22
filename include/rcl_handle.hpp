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

enum RclHandleType {
  RclHandleType_None,

  RclHandleType_ROSNode,
  RclHandleType_ROSPublisher,
  RclHandleType_ROSSubscription,
  RclHandleType_ROSService,
  RclHandleType_ROSClient,
  RclHandleType_ROSIDLString,
  RclHandleType_Malloc,

  RclHandleType_Count
};

class RclHandle : public Nan::ObjectWrap {
 public:
  static void Init(v8::Local<v8::Object> exports);
  static v8::Local<v8::Object> NewInstance(void* handle,
    RclHandleType type = RclHandleType_Malloc);

  void* GetPtr() { return pointer_; }
  void SetPtr(void* ptr) { pointer_ = ptr; }
  RclHandleType GetType() { return type_; }
  void SetType(RclHandleType type) { type_ = type; }

 private:
  RclHandle();
  ~RclHandle();

  void DestroyMe();

  static Nan::Persistent<v8::Function> constructor;
  static void New(const Nan::FunctionCallbackInfo<v8::Value>& info);
  static NAN_METHOD(Destroy);
  static NAN_METHOD(Dismiss);

  static NAN_GETTER(HandleGetter);
  static NAN_GETTER(TypeGetter);

 private:
  void* pointer_;
  RclHandleType type_;
};

}  // namespace rclnodejs

#endif
