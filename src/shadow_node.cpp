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

#include "shadow_node.hpp"

#include <memory>
#include <vector>

#include "executor.hpp"
#include "handle_manager.hpp"
#include "rcl_handle.hpp"

namespace rclnodejs {

Nan::Persistent<v8::Function> ShadowNode::constructor;

ShadowNode::ShadowNode() : handle_manager_(std::make_unique<HandleManager>()) {
  executor_ = std::make_unique<Executor>(handle_manager_.get(), this);
}

ShadowNode::~ShadowNode() {
  Nan::HandleScope scope;

  StopRunning();
}

void ShadowNode::Init(v8::Local<v8::Object> exports) {
  Nan::HandleScope scope;

  // Prepare constructor template
  v8::Local<v8::FunctionTemplate> tpl = Nan::New<v8::FunctionTemplate>(New);
  tpl->SetClassName(Nan::New("ShadowNode").ToLocalChecked());
  tpl->InstanceTemplate()->SetInternalFieldCount(1);

  Nan::SetPrototypeMethod(tpl, "start", Start);
  Nan::SetPrototypeMethod(tpl, "stop", Stop);
  Nan::SetPrototypeMethod(tpl, "syncHandles", SyncHandles);
  Nan::SetPrototypeMethod(tpl, "spinOnce", SpinOnce);

  v8::Local<v8::Context> context = exports->GetIsolate()->GetCurrentContext();

  constructor.Reset(tpl->GetFunction(context).ToLocalChecked());
  Nan::Set(exports, Nan::New("ShadowNode").ToLocalChecked(),
           tpl->GetFunction(context).ToLocalChecked());
}

void ShadowNode::StopRunning() {
  executor_->Stop();
  handle_manager_->ClearHandles();
}

void ShadowNode::StartRunning(rcl_context_t* context, int32_t timeout) {
  handle_manager_->SynchronizeHandles(this->handle());
  executor_->Start(context, timeout);
}

void ShadowNode::RunOnce(rcl_context_t* context, int32_t timeout) {
  handle_manager_->SynchronizeHandles(this->handle());
  executor_->SpinOnce(context, timeout);
}

NAN_METHOD(ShadowNode::Start) {
  auto* me = Nan::ObjectWrap::Unwrap<ShadowNode>(info.Holder());
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  auto timeout = Nan::To<int32_t>(info[1]).FromJust();
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  if (me) me->StartRunning(context, timeout);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ShadowNode::Stop) {
  auto* me = Nan::ObjectWrap::Unwrap<ShadowNode>(info.Holder());
  if (me) me->StopRunning();

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ShadowNode::SpinOnce) {
  auto* me = Nan::ObjectWrap::Unwrap<ShadowNode>(info.Holder());
  RclHandle* context_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  auto timeout = Nan::To<int32_t>(info[1]).FromJust();
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  if (me) me->RunOnce(context, timeout);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ShadowNode::SyncHandles) {
  auto* me = Nan::ObjectWrap::Unwrap<ShadowNode>(info.Holder());
  if (me) {
    me->handle_manager()->SynchronizeHandles(me->handle());
  }
}

void ShadowNode::Execute(const std::vector<rclnodejs::RclHandle*>& handles) {
  Nan::HandleScope scope;
  Nan::AsyncResource res("shadow_node");

  v8::Local<v8::Array> results = Nan::New<v8::Array>(handles.size());
  for (size_t i = 0; i < handles.size(); ++i) {
    handles[i]->SyncProperties();
    Nan::Set(results, i, handles[i]->handle());
  }

  v8::Local<v8::Value> argv[] = {results};

  res.runInAsyncScope(Nan::New(this->persistent()), "execute", 1, argv);
}

void ShadowNode::CatchException(std::exception_ptr e_ptr) {
  try {
    std::rethrow_exception(e_ptr);
  } catch (const std::exception& e) {
    Nan::ThrowError(e.what());
  }
}

void ShadowNode::New(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.IsConstructCall()) {
    // Invoked as constructor: `new ShadowNode(...)`
    ShadowNode* obj = new ShadowNode();
    obj->Wrap(info.This());
    info.GetReturnValue().Set(info.This());
  }
}

}  // namespace rclnodejs
