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

#include "rcl_handle.hpp"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

namespace rclnodejs {

Nan::Persistent<v8::Function> RclHandle::constructor;

RclHandle::RclHandle() : pointer_(nullptr), parent_(nullptr) {}

RclHandle::~RclHandle() {
  if (pointer_) Reset();
}

void RclHandle::Init(v8::Local<v8::Object> exports) {
  v8::Local<v8::FunctionTemplate> tpl = Nan::New<v8::FunctionTemplate>(New);
  tpl->SetClassName(Nan::New("RclHandle").ToLocalChecked());
  tpl->InstanceTemplate()->SetInternalFieldCount(1);

  Nan::SetAccessor(tpl->InstanceTemplate(),
                   Nan::New("properties").ToLocalChecked(), PropertiesGetter);
  Nan::SetPrototypeMethod(tpl, "release", Release);
  Nan::SetPrototypeMethod(tpl, "dismiss", Dismiss);

  v8::Local<v8::Context> context = exports->GetIsolate()->GetCurrentContext();

  constructor.Reset(tpl->GetFunction(context).ToLocalChecked());
  Nan::Set(exports, Nan::New("RclHandle").ToLocalChecked(),
           tpl->GetFunction(context).ToLocalChecked());
}

void RclHandle::New(const Nan::FunctionCallbackInfo<v8::Value>& info) {
  if (info.IsConstructCall()) {
    RclHandle* obj = new RclHandle();
    obj->Wrap(info.This());
    info.GetReturnValue().Set(info.This());
  }
}

void RclHandle::SyncProperties() {
  auto obj = v8::Object::New(v8::Isolate::GetCurrent());

  for (auto it = properties_.begin(); it != properties_.end(); it++) {
    Nan::Set(obj, Nan::New(it->first).ToLocalChecked(), Nan::New(it->second));
  }

  properties_obj_ = obj;
}

NAN_GETTER(RclHandle::PropertiesGetter) {
  auto* me = RclHandle::Unwrap<RclHandle>(info.Holder());

  if (!me->properties_obj_.IsEmpty())
    info.GetReturnValue().Set(me->properties_obj_);
  else
    info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(RclHandle::Release) {
  auto* me = Nan::ObjectWrap::Unwrap<RclHandle>(info.Holder());
  if (me->ptr()) me->Reset();

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(RclHandle::Dismiss) {
  auto* me = Nan::ObjectWrap::Unwrap<RclHandle>(info.Holder());
  if (me) me->set_ptr(nullptr);

  info.GetReturnValue().Set(Nan::Undefined());
}

v8::Local<v8::Object> RclHandle::NewInstance(
    void* handle, RclHandle* parent, std::function<void(void*)> deleter) {
  Nan::EscapableHandleScope scope;

  v8::Local<v8::Function> cons = Nan::New<v8::Function>(constructor);
  v8::Local<v8::Context> context =
      v8::Isolate::GetCurrent()->GetCurrentContext();

  v8::Local<v8::Object> instance =
      cons->NewInstance(context, 0, nullptr).ToLocalChecked();

  auto* rcl_handle = Nan::ObjectWrap::Unwrap<RclHandle>(instance);
  rcl_handle->set_ptr(handle);
  rcl_handle->set_deleter(deleter);
  if (parent) {
    rcl_handle->set_parent(parent);
    parent->AddChild(rcl_handle);
  }

  return scope.Escape(instance);
}

void RclHandle::Reset() {
  if (!pointer_) return;

  if (parent_) {
    parent_->RemoveChild(this);
    parent_ = nullptr;
  }

  for (auto* child : children_) {
    // Because the parent is going to reset the child, and don't want be
    // notified back from the child again, set the |parent_| of child to
    // nullptr.
    child->set_parent(nullptr);
    child->Reset();
  }

  if (deleter_) deleter_(pointer_);

  pointer_ = nullptr;
  children_.clear();
}

}  // namespace rclnodejs
