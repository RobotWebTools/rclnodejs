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

#ifndef SRC_RCL_HANDLE_HPP_
#define SRC_RCL_HANDLE_HPP_
#include <napi.h>

#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>
namespace rclnodejs {

class RclHandle : public Napi::ObjectWrap<RclHandle> {
 public:
  static Napi::Object Init(Napi::Env env, Napi::Object exports);
  static Napi::Object NewInstance(Napi::Env env, void* handle,
                                  RclHandle* parent,
                                  std::function<void(void*)> deleter);

  void set_deleter(std::function<void(void*)> deleter) { deleter_ = deleter; }

  RclHandle* parent() { return parent_; }
  void set_parent(RclHandle* parent) { parent_ = parent; }

  void* ptr() { return pointer_; }
  void set_ptr(void* ptr) { pointer_ = ptr; }

  void Reset();
  void AddChild(RclHandle* child) { children_.insert(child); }
  void RemoveChild(RclHandle* child) { children_.erase(child); }
  // void SetBoolProperty(const std::string& name, bool value);
  void SyncProperties();

  explicit RclHandle(const Napi::CallbackInfo& info);
  ~RclHandle();

 public:
  // Methods
  Napi::Value Release(const Napi::CallbackInfo& info);
  Napi::Value Dismiss(const Napi::CallbackInfo& info);

  // Property getter
  Napi::Value PropertiesGetter(const Napi::CallbackInfo& info);

  // Store property updates to be applied later
  struct PropertyUpdate {
    std::string name;
    bool value;
  };

  std::mutex property_mutex_;
  std::vector<PropertyUpdate> pending_property_updates_;

  // ThreadSafeFunction to process updates on main thread
  static Napi::ThreadSafeFunction tsfn_;

  // Thread-safe version to queue property updates
  //   void SetBoolProperty(const std::string& name, bool value) {

  //     std::lock_guard<std::mutex> lock(property_mutex_);
  //     pending_property_updates_.push_back({name, value});
  //     if (tsfn_) {
  //         // Queue work to be processed on main thread
  //         tsfn_.BlockingCall(this, []( Napi::Env env, Napi::Function
  //         jsCallback, RclHandle* handle) {
  //           std::cout << "====ProcessPropertyUpdates" << std::endl;
  //           handle->ProcessPropertyUpdates();
  //         });
  //         // napi_status status = tsfn_.BlockingCall(this, [](Napi::Env env,
  //         Napi::Function jsCallback, RclHandle* handle) {
  //         //   std::cout << "====SetBoolProperty" << std::endl;
  //         // });
  //         // if ( status != napi_ok )
  //         // {
  //         //   std::cout << "===ERRRRRRRRRRRRRrr" << std::endl;
  //         // }
  //     }
  // }
  void SetBoolProperty(const std::string& name, bool value) {
    properties_[name] = value;
  }
  // Process pending property updates on main thread
  void ProcessPropertyUpdates() {
    std::vector<PropertyUpdate> updates;
    {
      std::lock_guard<std::mutex> lock(property_mutex_);
      updates.swap(pending_property_updates_);
    }

    Napi::Env env = Env();  // Get env from ObjectWrap
    for (const auto& update : updates) {
      if (!properties_obj_.IsEmpty()) {
        Napi::Object props = properties_obj_.Value().As<Napi::Object>();
        props.Set(update.name, Napi::Boolean::New(env, update.value));
      }
    }
  }

  // Initialize thread-safe function
  static void InitThreadSafeFunction(Napi::Env env) {
    if (!tsfn_) {
      // std::cout << "====InitThreadSafeFunction" << std::endl;
      tsfn_ = Napi::ThreadSafeFunction::New(
          env,
          Napi::Function::New(env,
                              [](const Napi::CallbackInfo& info) {
                                // This runs on the main thread
                                // Napi::Env env = info.Env();
                                // RclHandle* handle =
                                // RclHandle::Unwrap(info[0].As<Napi::Object>());
                                // handle->ProcessPropertyUpdates();
                                // std::cout << "====ProcessPropertyUpdates" <<
                                // std::endl;
                              }),
          "SetBoolProperty",
          0,   // Max queue size (0 = unlimited)
          1);  // Number of threads to use);
    }
  }

  // Finalize the thread-safe function
  static void CleanupThreadSafeFunction() {
    if (tsfn_) {
      tsfn_.Release();
    }
  }

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

#endif  // SRC_RCL_HANDLE_HPP_
