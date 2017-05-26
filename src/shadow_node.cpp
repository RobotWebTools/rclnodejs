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

namespace rclnodejs {

Nan::Persistent<v8::Function> ShadowNode::constructor;

ShadowNode::ShadowNode() {
  handle_manager_ = std::make_unique<HandleManager>();
  executor_ = std::make_unique<Executor>(handle_manager_.get(), this);
}

ShadowNode::~ShadowNode() {
  executor_->Stop();

  Nan::HandleScope scope;
  v8::Local<v8::Value> argv[0];
  Nan::MakeCallback(Nan::New(this->persistent()), "destoryNode", 0, argv);
}

void ShadowNode::Init(v8::Local<v8::Object> exports) {
  Nan::HandleScope scope;

  // Prepare constructor template
  v8::Local<v8::FunctionTemplate> tpl = Nan::New<v8::FunctionTemplate>(New);
  tpl->SetClassName(Nan::New("ShadowNode").ToLocalChecked());
  tpl->InstanceTemplate()->SetInternalFieldCount(1);


  constructor.Reset(tpl->GetFunction());
  exports->Set(Nan::New("ShadowNode").ToLocalChecked(), tpl->GetFunction());
}

void ShadowNode::Spin() {
  handle_manager_->CollectHandles(this->handle());
  executor_->Start();
}

void ShadowNode::Shutdown() {
  executor_->Stop();
}

void ShadowNode::Execute() {
  Nan::HandleScope scope;
  v8::Local<v8::Value> argv[0];
  Nan::MakeCallback(Nan::New(this->persistent()), "execute", 0, argv);
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
