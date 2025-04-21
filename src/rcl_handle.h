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

#ifndef SRC_RCL_HANDLE_H_
#define SRC_RCL_HANDLE_H_

#include <napi.h>

#include <functional>
#include <map>
#include <set>
#include <string>

namespace rclnodejs {

class RclHandle : public Napi::ObjectWrap<RclHandle> {
 public:
  static Napi::Object Init(Napi::Env env, Napi::Object exports);
  static Napi::Object NewInstance(Napi::Env env, void* handle,
                                  RclHandle* parent,
                                  std::function<void(void*)> deleter);

  explicit RclHandle(const Napi::CallbackInfo& info);
  ~RclHandle();

  void set_deleter(std::function<void(void*)> deleter) { deleter_ = deleter; }

  RclHandle* parent() { return parent_; }
  void set_parent(RclHandle* parent) { parent_ = parent; }

  void* ptr() { return pointer_; }
  void set_ptr(void* ptr) { pointer_ = ptr; }

  void Reset();
  void AddChild(RclHandle* child) { children_.insert(child); }
  void RemoveChild(RclHandle* child) { children_.erase(child); }
  void SetBoolProperty(const std::string& name, bool value) {
    properties_[name] = value;
  }
  void SyncProperties();

 private:
  Napi::Value Release(const Napi::CallbackInfo& info);
  Napi::Value Dismiss(const Napi::CallbackInfo& info);
  Napi::Value PropertiesGetter(const Napi::CallbackInfo& info);

 private:
  void* pointer_;
  RclHandle* parent_;
  std::map<std::string, bool> properties_;
  Napi::ObjectReference properties_obj_;

  std::function<void(void*)> deleter_;
  std::set<RclHandle*> children_;

  static Napi::FunctionReference constructor;
};

}  // namespace rclnodejs

#endif  // SRC_RCL_HANDLE_H_
