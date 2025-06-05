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

#include "shadow_node.h"

#include <memory>
#include <vector>

#include "executor.h"
#include "handle_manager.h"
#include "rcl_handle.h"

namespace rclnodejs {

Napi::FunctionReference ShadowNode::constructor;

ShadowNode::ShadowNode(const Napi::CallbackInfo& info)
    : Napi::ObjectWrap<ShadowNode>(info),
      handle_manager_(std::make_unique<HandleManager>()) {
  executor_ =
      std::make_unique<Executor>(info.Env(), handle_manager_.get(), this);
}

ShadowNode::~ShadowNode() { StopRunning(); }

void ShadowNode::Init(Napi::Env env, Napi::Object exports) {
  Napi::HandleScope scope(env);

  Napi::Function func =
      DefineClass(env, "ShadowNode",
                  {
                      InstanceMethod("start", &ShadowNode::Start),
                      InstanceMethod("stop", &ShadowNode::Stop),
                      InstanceMethod("syncHandles", &ShadowNode::SyncHandles),
                      InstanceMethod("spinOnce", &ShadowNode::SpinOnce),
                  });

  constructor = Napi::Persistent(func);
  constructor.SuppressDestruct();
  exports.Set("ShadowNode", func);
}

void ShadowNode::StopRunning() {
  executor_->Stop();
  handle_manager_->ClearHandles();
}

void ShadowNode::StartRunning(rcl_context_t* context, int32_t timeout) {
  handle_manager_->SynchronizeHandles(this->Value());
  executor_->Start(context, timeout);
}

void ShadowNode::RunOnce(rcl_context_t* context, int32_t timeout) {
  handle_manager_->SynchronizeHandles(this->Value());
  executor_->SpinOnce(context, timeout);
}

Napi::Value ShadowNode::Start(const Napi::CallbackInfo& info) {
  Napi::Object context_handle = info[0].As<Napi::Object>();
  RclHandle* handle = RclHandle::Unwrap(context_handle);
  int32_t timeout = info[1].As<Napi::Number>().Int32Value();

  rcl_context_t* context = reinterpret_cast<rcl_context_t*>(handle->ptr());
  StartRunning(context, timeout);

  return info.Env().Undefined();
}

Napi::Value ShadowNode::Stop(const Napi::CallbackInfo& info) {
  StopRunning();
  return info.Env().Undefined();
}

Napi::Value ShadowNode::SpinOnce(const Napi::CallbackInfo& info) {
  Napi::Object context_handle = info[0].As<Napi::Object>();
  RclHandle* handle = RclHandle::Unwrap(context_handle);
  int32_t timeout = info[1].As<Napi::Number>().Int32Value();

  rcl_context_t* context = reinterpret_cast<rcl_context_t*>(handle->ptr());
  RunOnce(context, timeout);

  return info.Env().Undefined();
}

Napi::Value ShadowNode::SyncHandles(const Napi::CallbackInfo& info) {
  handle_manager()->SynchronizeHandles(this->Value());
  return info.Env().Undefined();
}

void ShadowNode::Execute(const std::vector<rclnodejs::RclHandle*>& handles) {
  Napi::Env env = Env();
  Napi::HandleScope scope(env);

  Napi::Array results = Napi::Array::New(env, handles.size());
  for (size_t i = 0; i < handles.size(); ++i) {
    handles[i]->SyncProperties();
    results[i] = handles[i]->Value();
  }

  Napi::Function execute =
      Value().As<Napi::Object>().Get("execute").As<Napi::Function>();
  execute.Call(Value(), {results});
}

void ShadowNode::CatchException(std::exception_ptr e_ptr) {
  try {
    std::rethrow_exception(e_ptr);
  } catch (const std::exception& e) {
    Napi::Error::New(Env(), e.what()).ThrowAsJavaScriptException();
  }
}

}  // namespace rclnodejs
