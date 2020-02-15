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

#ifndef RCLNODEJS_HANDLE_MANAGER_HPP_
#define RCLNODEJS_HANDLE_MANAGER_HPP_

#include <nan.h>
#include <rcl/wait.h>

#include <atomic>
#include <vector>

#include "rcl_handle.hpp"

namespace rclnodejs {

class ScopedMutex {
 public:
  explicit ScopedMutex(uv_mutex_t* mutex) : mutex_(mutex) {
    uv_mutex_lock(mutex_);
  }
  ~ScopedMutex() { uv_mutex_unlock(mutex_); }
  uv_mutex_t* mutex_;
};

class HandleManager {
 public:
  HandleManager();
  ~HandleManager();

  void CollectHandles(const v8::Local<v8::Object> node);
  bool AddHandlesToWaitSet(rcl_wait_set_t* wait_set);
  void CollectReadyHandles(rcl_wait_set_t* wait_set);
  void ClearHandles();
  void WaitForSynchronizing() { uv_sem_wait(&sem_); }

  uint32_t subscription_count() const { return subscriptions_.size(); }
  uint32_t service_count() const { return services_.size(); }
  uint32_t client_count() const { return clients_.size(); }
  uint32_t timer_count() const { return  timers_.size(); }
  uint32_t guard_condition_count() const { return guard_conditions_.size(); }
  std::vector<rclnodejs::RclHandle*>
      get_ready_handles() const { return ready_handles_; }
  uv_mutex_t* mutex() { return &mutex_; }
  bool is_synchronizing() const { return is_synchronizing_.load(); }
  bool is_empty() const { return subscriptions_.size() == 0
      && services_.size() == 0
      && clients_.size() == 0
      && timers_.size() == 0
      && guard_conditions_.size() == 0; }

 protected:
  void CollectHandlesByType(const v8::Local<v8::Object>& typeObject,
                            std::vector<rclnodejs::RclHandle*>* vec);
  template<typename T> void CollectReadyHandlesByType(
      const T** struct_ptr,
      size_t size,
      const std::vector<rclnodejs::RclHandle*>& handles);

 private:
  std::vector<rclnodejs::RclHandle*> timers_;
  std::vector<rclnodejs::RclHandle*> clients_;
  std::vector<rclnodejs::RclHandle*> services_;
  std::vector<rclnodejs::RclHandle*> subscriptions_;
  std::vector<rclnodejs::RclHandle*> guard_conditions_;
  std::vector<rclnodejs::RclHandle*> ready_handles_;

  uv_mutex_t mutex_;
  uv_sem_t sem_;
  std::atomic_bool is_synchronizing_;
};

}  // namespace rclnodejs

#endif
