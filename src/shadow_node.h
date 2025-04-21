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

#ifndef SRC_SHADOW_NODE_H_
#define SRC_SHADOW_NODE_H_

#include <napi.h>

#include <exception>
#include <memory>
#include <vector>

#include "executor.h"

namespace rclnodejs {

class HandleManager;
class Executor;

class ShadowNode : public Napi::ObjectWrap<ShadowNode>,
                   public Executor::Delegate {
 public:
  static void Init(Napi::Env env, Napi::Object exports);
  void StartRunning(rcl_context_t* context, int32_t timeout);
  void StopRunning();
  void RunOnce(rcl_context_t* context, int32_t timeout);

  HandleManager* handle_manager() { return handle_manager_.get(); }

  // Executor::Delegate overrides:
  void Execute(const std::vector<rclnodejs::RclHandle*>& handles) override;
  void CatchException(std::exception_ptr e_ptr) override;

  explicit ShadowNode(const Napi::CallbackInfo& info);
  ~ShadowNode();

 private:
  static Napi::FunctionReference constructor;

  Napi::Value Stop(const Napi::CallbackInfo& info);
  Napi::Value Start(const Napi::CallbackInfo& info);
  Napi::Value SyncHandles(const Napi::CallbackInfo& info);
  Napi::Value SpinOnce(const Napi::CallbackInfo& info);
  Napi::Value HandleGetter(const Napi::CallbackInfo& info);
  void HandleSetter(const Napi::CallbackInfo& info, const Napi::Value& value);

  std::unique_ptr<HandleManager> handle_manager_;
  std::unique_ptr<Executor> executor_;
};

}  // namespace rclnodejs

#endif  // SRC_SHADOW_NODE_H_
