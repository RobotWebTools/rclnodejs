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

#include "executor.hpp"

namespace rclnodejs {

class HandleManager;
class Executor;

class ShadowNode : public Nan::ObjectWrap,
                   public Executor::Delegate {
 public:
  static void Init(v8::Local<v8::Object> exports);
  void Spin();
  void Shutdown();

  // Executor::Delegate overrides:
  void Execute() override;
  void CatchException(std::exception_ptr e_ptr) override;

 private:
  ShadowNode();
  ~ShadowNode();

  static void New(const Nan::FunctionCallbackInfo<v8::Value>& info);
  static Nan::Persistent<v8::Function> constructor;

  std::unique_ptr<HandleManager> handle_manager_;
  std::unique_ptr<Executor> executor_;
};

}  // namespace rclnodejs

#endif
