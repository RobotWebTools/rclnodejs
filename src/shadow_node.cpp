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

#include "executor.hpp"
#include "handle_manager.hpp"
#include "rcl_handle.hpp"

namespace rclnodejs {

Nan::Persistent<v8::Function> ShadowNode::constructor;

ShadowNode::ShadowNode() : rcl_handle_(nullptr) {
  handle_manager_ = std::make_unique<HandleManager>();
  executor_ = std::make_unique<Executor>(handle_manager_.get(), this);
  rcl_handle_.reset(new Nan::Persistent<v8::Object>());
}

ShadowNode::~ShadowNode() {
  StopRunning();
  if (!rcl_handle_->IsEmpty()) {
    RclHandle* handle = RclHandle::Unwrap<RclHandle>(Nan::New(*rcl_handle_));
    handle->Reset();
  }
}

void ShadowNode::Init(v8::Local<v8::Object> exports) {
  Nan::HandleScope scope;

  // Prepare constructor template
  v8::Local<v8::FunctionTemplate> tpl = Nan::New<v8::FunctionTemplate>(New);
  tpl->SetClassName(Nan::New("ShadowNode").ToLocalChecked());
  tpl->InstanceTemplate()->SetInternalFieldCount(1);

  Nan::SetAccessor(tpl->InstanceTemplate(), Nan::New("handle").ToLocalChecked(),
                   HandleGetter, HandleSetter);
  Nan::SetPrototypeMethod(tpl, "start", Start);
  Nan::SetPrototypeMethod(tpl, "stop", Stop);
  Nan::SetPrototypeMethod(tpl, "syncHandles", SyncHandles);

  constructor.Reset(tpl->GetFunction());
  exports->Set(Nan::New("ShadowNode").ToLocalChecked(), tpl->GetFunction());
}

NAN_GETTER(ShadowNode::HandleGetter) {
  auto* me = ShadowNode::Unwrap<ShadowNode>(info.Holder());

  if (!me->rcl_handle()->IsEmpty())
    info.GetReturnValue().Set(Nan::New(*(me->rcl_handle())));
  else
    info.GetReturnValue().Set(Nan::Undefined());
}

NAN_SETTER(ShadowNode::HandleSetter) {
  auto* me = ShadowNode::Unwrap<ShadowNode>(info.Holder());
  if (value->ToObject()->InternalFieldCount() > 0)
    me->rcl_handle()->Reset(value->ToObject());
}

void ShadowNode::StopRunning() {
  executor_->Stop();
  handle_manager_->ClearHandles();
}

void ShadowNode::StartRunning() {
  handle_manager_->CollectHandles(this->handle());
  executor_->Start();
}

NAN_METHOD(ShadowNode::Start) {
  auto* me = Nan::ObjectWrap::Unwrap<ShadowNode>(info.Holder());
  if (me)
    me->StartRunning();

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ShadowNode::Stop) {
  auto* me = Nan::ObjectWrap::Unwrap<ShadowNode>(info.Holder());
  if (me)
    me->StopRunning();

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ShadowNode::SyncHandles) {
  auto* me = Nan::ObjectWrap::Unwrap<ShadowNode>(info.Holder());
  if (me) {
    me->handle_manager()->CollectHandles(me->handle());
  }
}

void ShadowNode::Execute() {
  Nan::HandleScope scope;
  v8::Local<v8::Value> argv[1];
  Nan::AsyncResource res("shadow_node");
  res.runInAsyncScope(Nan::New(this->persistent()), "execute", 0, argv);
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
