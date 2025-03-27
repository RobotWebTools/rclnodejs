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

#include <iostream>

#include "macros.hpp"

namespace rclnodejs {

Napi::FunctionReference RclHandle::constructor;

// Define the static ThreadSafeFunction
Napi::ThreadSafeFunction RclHandle::tsfn_;

RclHandle::RclHandle(const Napi::CallbackInfo& info)
    : Napi::ObjectWrap<RclHandle>(info), pointer_(nullptr), parent_(nullptr) {}

RclHandle::~RclHandle() {
  if (pointer_) Reset();

  // tsfn_.Release();
}

Napi::Object RclHandle::Init(Napi::Env env, Napi::Object exports) {
  Napi::HandleScope scope(env);

  Napi::Function func = DefineClass(
      env, "RclHandle",
      {InstanceMethod("release", &RclHandle::Release),
       InstanceMethod("dismiss", &RclHandle::Dismiss),
       InstanceAccessor("properties", &RclHandle::PropertiesGetter, nullptr)});

  constructor = Napi::Persistent(func);
  constructor.SuppressDestruct();
  exports.Set("RclHandle", func);

  return exports;
}

void RclHandle::SyncProperties() {
  // Napi::Env env = Env();
  Napi::Env env = rclnodejs::GetEnv();
  // if (env.IsNull()) return;
  // std::cout << "===before SyncProperties" << std::endl;

  Napi::HandleScope scope(env);
  // std::cout << "====after SyncProperties" << std::endl;
  Napi::Object obj = Napi::Object::New(env);
  // std::cout << "after====after SyncProperties" << std::endl;

  for (auto& pair : properties_) {
    obj.Set(pair.first, Napi::Boolean::New(env, pair.second));
  }

  properties_obj_ = Napi::Persistent(obj);
}

// void RclHandle::SetBoolProperty(const std::string& name, bool value) {
//   // properties_[name] = value;
//   // SyncProperties();
//   std::lock_guard<std::mutex> lock(property_mutex_);

//   // Allocate a new update object
//   PropertyUpdate* update = new PropertyUpdate{name, value};

//   // Queue the update to be executed on the main thread
//   napi_status status = tsfn_.BlockingCall(update);
//   if (status != napi_ok) {
//     delete update;  // Clean up if failed
//     // Handle error appropriately (log or handle internally)
//   }
// }

Napi::Value RclHandle::PropertiesGetter(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  if (!properties_obj_.IsEmpty())
    return properties_obj_.Value();
  else
    return env.Undefined();
}

Napi::Value RclHandle::Release(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  if (ptr()) Reset();
  return env.Undefined();
}

Napi::Value RclHandle::Dismiss(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  set_ptr(nullptr);
  return env.Undefined();
}

Napi::Object RclHandle::NewInstance(Napi::Env env, void* handle,
                                    RclHandle* parent,
                                    std::function<void(void*)> deleter) {
  Napi::EscapableHandleScope scope(env);

  Napi::Object instance = constructor.New({});

  RclHandle* rcl_handle = Napi::ObjectWrap<RclHandle>::Unwrap(instance);
  rcl_handle->set_ptr(handle);
  rcl_handle->set_deleter(deleter);
  if (parent) {
    rcl_handle->set_parent(parent);
    parent->AddChild(rcl_handle);
  }

  return scope.Escape(instance).As<Napi::Object>();
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
