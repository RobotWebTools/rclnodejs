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

class ScopedReadWriteLock {
 public:
  enum class LockType { kWrite, kRead };

  ScopedReadWriteLock(uv_rwlock_t* rwlock, LockType type)
      : rwlock_(rwlock), type_(type) {
    if (type_ == LockType::kWrite)
      uv_rwlock_wrlock(rwlock_);
    else
      uv_rwlock_rdlock(rwlock_);
  }

  ~ScopedReadWriteLock() {
    if (type_ == LockType::kWrite)
      uv_rwlock_wrunlock(rwlock_);
    else
      uv_rwlock_rdunlock(rwlock_);
  }

  uv_rwlock_t* rwlock_;
  LockType type_;
};

class HandleManager {
 public:
  HandleManager();
  ~HandleManager();

  void SynchronizeHandles(const v8::Local<v8::Object> node);
  void WaitForSynchronizing();

  // Waits the handles to be attached from the background thread.
  void WaitForHandles();
  // Signals to stop waiting the handles from the main thread.
  void StopWaitingHandles();

  void ClearHandles();

  rcl_ret_t AddHandlesToWaitSet(rcl_wait_set_t* wait_set);
  rcl_ret_t CollectReadyHandles(rcl_wait_set_t* wait_set);
  rcl_ret_t GetEntityCounts(size_t* subscriptions_size,
                            size_t* guard_conditions_size, size_t* timers_size,
                            size_t* clients_size, size_t* services_size);

  uint32_t subscription_count() const { return subscriptions_.size(); }
  uint32_t service_count() const { return services_.size(); }
  uint32_t client_count() const { return clients_.size(); }
  uint32_t timer_count() const { return timers_.size(); }
  uint32_t guard_condition_count() const { return guard_conditions_.size(); }
  uv_rwlock_t* handle_rwlock() { return &sync_handles_rwlock_; }

  uint32_t ready_handles_count();

  // Takes the ownership of the handles that are ready to be taken.
  std::vector<rclnodejs::RclHandle*> TakeReadyHandles();

  // Thread safe function to get if the handles are being synchronized.
  bool is_synchronizing() const { return is_synchronizing_.load(); }

  // Thread safe function to get the sum of the handles.
  uint32_t sum() const { return sum_; }

 protected:
  // Synchronize the handles from `typeObject`.
  uint32_t SynchronizeHandlesByType(const v8::Local<v8::Object>& typeObject,
                                    std::vector<rclnodejs::RclHandle*>* vec);
  template <typename T>
  void CollectReadyHandlesByType(
      const T** struct_ptr, size_t size,
      const std::vector<rclnodejs::RclHandle*>& handles,
      std::vector<rclnodejs::RclHandle*>* ready_handles);
  rcl_ret_t CollectReadyActionHandles(
      rcl_wait_set_t* wait_set,
      std::vector<rclnodejs::RclHandle*>* ready_handles);

 private:
  std::vector<rclnodejs::RclHandle*> timers_;
  std::vector<rclnodejs::RclHandle*> clients_;
  std::vector<rclnodejs::RclHandle*> services_;
  std::vector<rclnodejs::RclHandle*> subscriptions_;
  std::vector<rclnodejs::RclHandle*> guard_conditions_;
  std::vector<rclnodejs::RclHandle*> action_servers_;
  std::vector<rclnodejs::RclHandle*> action_clients_;
  std::vector<rclnodejs::RclHandle*> ready_handles_;

  // Protects the handles.
  uv_rwlock_t sync_handles_rwlock_;
  uv_rwlock_t ready_handles_rwlock_;

  // Used to wait for handles added from the background thread.
  uv_sem_t wait_handle_sem_;

  // Used to wait from the background thread when the handles are being
  // synchronized on the main thread.
  uv_sem_t sync_handle_sem_;

  // Atomically written from the main thread only, but read from background
  // thread.
  std::atomic_bool is_synchronizing_;

  // Sum of the handles attached to the current node, atomically written from
  // the main thread.
  std::atomic_uint32_t sum_;
};

}  // namespace rclnodejs

#endif
