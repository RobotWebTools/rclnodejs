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

#ifndef RCLNODEJS_SHADOW_NODE_HPP_
#define RCLNODEJS_SHADOW_NODE_HPP_

#include <nan.h>

#include <exception>
#include <memory>
#include <vector>

#include "executor.hpp"

namespace rclnodejs {

class HandleManager;
class Executor;

class ShadowNode : public Nan::ObjectWrap, public Executor::Delegate {
 public:
  static void Init(v8::Local<v8::Object> exports);
  void StartRunning(rcl_context_t* context, int32_t timeout);
  void StopRunning();
  void RunOnce(rcl_context_t* context, int32_t timeout);

  HandleManager* handle_manager() { return handle_manager_.get(); }

  // Executor::Delegate overrides:
  void Execute(const std::vector<rclnodejs::RclHandle*>& handles) override;
  void CatchException(std::exception_ptr e_ptr) override;

 private:
  ShadowNode();
  ~ShadowNode();

  static void New(const Nan::FunctionCallbackInfo<v8::Value>& info);
  static Nan::Persistent<v8::Function> constructor;
  static NAN_METHOD(Stop);
  static NAN_METHOD(Start);
  static NAN_METHOD(SyncHandles);
  static NAN_METHOD(SpinOnce);
  static NAN_GETTER(HandleGetter);
  static NAN_SETTER(HandleSetter);

  std::unique_ptr<HandleManager> handle_manager_;
  std::unique_ptr<Executor> executor_;
};

}  // namespace rclnodejs

#endif
