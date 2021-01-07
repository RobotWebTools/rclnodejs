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

#ifndef RCLNODEJS_RCL_HANDLE_HPP_
#define RCLNODEJS_RCL_HANDLE_HPP_

#include <nan.h>

#include <functional>
#include <map>
#include <set>
#include <string>

namespace rclnodejs {

class RclHandle : public Nan::ObjectWrap {
 public:
  static void Init(v8::Local<v8::Object> exports);
  static v8::Local<v8::Object> NewInstance(void* handle, RclHandle* parent,
                                           std::function<void(void*)> deleter);

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
  RclHandle();
  ~RclHandle();

  static Nan::Persistent<v8::Function> constructor;
  static void New(const Nan::FunctionCallbackInfo<v8::Value>& info);
  static NAN_METHOD(Release);
  static NAN_METHOD(Dismiss);
  static NAN_GETTER(PropertiesGetter);

 private:
  void* pointer_;
  RclHandle* parent_;
  std::map<std::string, bool> properties_;
  v8::Local<v8::Object> properties_obj_;

  std::function<void(void*)> deleter_;
  std::set<RclHandle*> children_;
};

}  // namespace rclnodejs

#endif
