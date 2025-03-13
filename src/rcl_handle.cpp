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

#include <node_api.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

namespace rclnodejs {

Napi::FunctionReference RclHandle::constructor;

RclHandle::RclHandle() : pointer_(nullptr), parent_(nullptr) {}

RclHandle::~RclHandle() {
  if (pointer_) Reset();
}

void RclHandle::Init(Napi::Env env, Napi::Object exports) {
  Napi::Function func = DefineClass(env, "RclHandle", {
    InstanceAccessor("properties", &RclHandle::PropertiesGetter, nullptr),
    InstanceMethod("release", &RclHandle::Release),
    InstanceMethod("dismiss", &RclHandle::Dismiss)
  });

  constructor = Napi::Persistent(func);
  constructor.SuppressDestruct();
  exports.Set("RclHandle", func);
}

void RclHandle::New(const Napi::CallbackInfo& info) {
  if (info.IsConstructCall()) {
    RclHandle* obj = new RclHandle();
    obj->Wrap(info.This());
    info.GetReturnValue().Set(info.This());
  }
}

void RclHandle::SyncProperties(napi_env env) {
  Napi::Object obj = Napi::Object::New(env);

  for (auto it = properties_.begin(); it != properties_.end(); it++) {
    obj.Set(it->first, it->second);
  }

  properties_obj_ = Napi::Persistent(obj);
}

Napi::Value RclHandle::PropertiesGetter(const Napi::CallbackInfo& info) {
  RclHandle* me = RclHandle::Unwrap<RclHandle>(info.Holder());

  if (!me->properties_obj_.IsEmpty())
    return me->properties_obj_.Value();
  else
    return info.Env().Undefined();
}

Napi::Value RclHandle::Release(const Napi::CallbackInfo& info) {
  RclHandle* me = RclHandle::Unwrap<RclHandle>(info.Holder());
  if (me->ptr()) me->Reset();

  return info.Env().Undefined();
}

Napi::Value RclHandle::Dismiss(const Napi::CallbackInfo& info) {
  RclHandle* me = RclHandle::Unwrap<RclHandle>(info.Holder());
  if (me) me->set_ptr(nullptr);

  return info.Env().Undefined();
}

Napi::Object RclHandle::NewInstance(napi_env env, void* handle, RclHandle* parent, std::function<void(void*)> deleter) {
  Napi::EscapableHandleScope scope(env);

  Napi::Object instance = constructor.New({});

  RclHandle* rcl_handle = Napi::ObjectWrap<RclHandle>::Unwrap(instance);
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
