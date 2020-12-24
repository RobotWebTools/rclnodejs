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

#include "handle_manager.hpp"

#include <rcl_action/rcl_action.h>

#include <utility>
#include <vector>

#include "macros.hpp"

namespace rclnodejs {

HandleManager::HandleManager() {
  is_synchronizing_.store(false);
  uv_rwlock_init(&sync_handles_rwlock_);
  uv_rwlock_init(&ready_handles_rwlock_);
  uv_sem_init(&sync_handle_sem_, 0);
  uv_sem_init(&wait_handle_sem_, 0);
}

HandleManager::~HandleManager() {
  uv_rwlock_destroy(&sync_handles_rwlock_);
  uv_rwlock_destroy(&ready_handles_rwlock_);
  uv_sem_destroy(&sync_handle_sem_);
  uv_sem_destroy(&wait_handle_sem_);
}

void HandleManager::SynchronizeHandles(const v8::Local<v8::Object> node) {
  Nan::HandleScope scope;
  Nan::MaybeLocal<v8::Value> timers =
      Nan::Get(node, Nan::New("_timers").ToLocalChecked());
  Nan::MaybeLocal<v8::Value> subscriptions =
      Nan::Get(node, Nan::New("_subscriptions").ToLocalChecked());
  Nan::MaybeLocal<v8::Value> clients =
      Nan::Get(node, Nan::New("_clients").ToLocalChecked());
  Nan::MaybeLocal<v8::Value> services =
      Nan::Get(node, Nan::New("_services").ToLocalChecked());
  Nan::MaybeLocal<v8::Value> guard_conditions =
      Nan::Get(node, Nan::New("_guards").ToLocalChecked());
  Nan::MaybeLocal<v8::Value> action_clients =
      Nan::Get(node, Nan::New("_actionClients").ToLocalChecked());
  Nan::MaybeLocal<v8::Value> action_servers =
      Nan::Get(node, Nan::New("_actionServers").ToLocalChecked());

  uint32_t sum = 0;
  is_synchronizing_.store(true);
  {
    ScopedReadWriteLock scoped_lock(&sync_handles_rwlock_,
                                    ScopedReadWriteLock::LockType::kWrite);
    ClearHandles();
    sum += SynchronizeHandlesByType(
        Nan::To<v8::Object>(timers.ToLocalChecked()).ToLocalChecked(),
        &timers_);
    sum += SynchronizeHandlesByType(
        Nan::To<v8::Object>(subscriptions.ToLocalChecked()).ToLocalChecked(),
        &subscriptions_);
    sum += SynchronizeHandlesByType(
        Nan::To<v8::Object>(clients.ToLocalChecked()).ToLocalChecked(),
        &clients_);
    sum += SynchronizeHandlesByType(
        Nan::To<v8::Object>(services.ToLocalChecked()).ToLocalChecked(),
        &services_);
    sum += SynchronizeHandlesByType(
        Nan::To<v8::Object>(guard_conditions.ToLocalChecked()).ToLocalChecked(),
        &guard_conditions_);
    sum += SynchronizeHandlesByType(
        Nan::To<v8::Object>(action_clients.ToLocalChecked()).ToLocalChecked(),
        &action_clients_);
    sum += SynchronizeHandlesByType(
        Nan::To<v8::Object>(action_servers.ToLocalChecked()).ToLocalChecked(),
        &action_servers_);
  }
  is_synchronizing_.store(false);

  // Signals that the synchronization has finished.
  uv_sem_post(&sync_handle_sem_);

  // Wakeup the backgroud thread if the sum was zero, but now it is greater than
  // zero.
  uint32_t sum_was = sum_.exchange(sum);
  if (sum_was == 0 && sum_ > 0) uv_sem_post(&wait_handle_sem_);

  RCLNODEJS_DEBUG(
      "Add %lu timers, %lu subscriptions, %lu clients, %lu services, %lu "
      "guards.",
      timers_.size(), subscriptions_.size(), clients_.size(), services_.size(),
      guard_conditions_.size());
}

void HandleManager::WaitForSynchronizing() { uv_sem_wait(&sync_handle_sem_); }

void HandleManager::WaitForHandles() { uv_sem_wait(&wait_handle_sem_); }

void HandleManager::StopWaitingHandles() { uv_sem_post(&wait_handle_sem_); }

void HandleManager::ClearHandles() {
  timers_.clear();
  clients_.clear();
  services_.clear();
  subscriptions_.clear();
  guard_conditions_.clear();
  action_clients_.clear();
  action_servers_.clear();
}

rcl_ret_t HandleManager::AddHandlesToWaitSet(rcl_wait_set_t* wait_set) {
  for (auto& timer : timers_) {
    rcl_timer_t* rcl_timer = reinterpret_cast<rcl_timer_t*>(timer->ptr());
    rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, rcl_timer, nullptr);
    if (ret != RCL_RET_OK) return ret;
  }

  for (auto& subscription : subscriptions_) {
    rcl_subscription_t* rcl_subscription =
        reinterpret_cast<rcl_subscription_t*>(subscription->ptr());
    rcl_ret_t ret =
        rcl_wait_set_add_subscription(wait_set, rcl_subscription, nullptr);
    if (ret != RCL_RET_OK) return ret;
  }

  for (auto& client : clients_) {
    rcl_client_t* rcl_client = reinterpret_cast<rcl_client_t*>(client->ptr());
    rcl_ret_t ret = rcl_wait_set_add_client(wait_set, rcl_client, nullptr);
    if (ret != RCL_RET_OK) return ret;
  }

  for (auto& service : services_) {
    rcl_service_t* rcl_service =
        reinterpret_cast<rcl_service_t*>(service->ptr());
    rcl_ret_t ret = rcl_wait_set_add_service(wait_set, rcl_service, nullptr);
    if (ret != RCL_RET_OK) return ret;
  }

  for (auto& guard_condition : guard_conditions_) {
    rcl_guard_condition_t* rcl_guard_condition =
        reinterpret_cast<rcl_guard_condition_t*>(guard_condition->ptr());
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(
        wait_set, rcl_guard_condition, nullptr);
    if (ret != RCL_RET_OK) return ret;
  }

  for (auto& action_client : action_clients_) {
    rcl_action_client_t* rcl_action_client =
        reinterpret_cast<rcl_action_client_t*>(action_client->ptr());
    if (rcl_action_wait_set_add_action_client(wait_set, rcl_action_client,
                                              nullptr, nullptr) != RCL_RET_OK)
      return false;
  }

  for (auto& action_server : action_servers_) {
    rcl_action_server_t* rcl_action_server =
        reinterpret_cast<rcl_action_server_t*>(action_server->ptr());
    rcl_ret_t ret = rcl_action_wait_set_add_action_server(
        wait_set, rcl_action_server, nullptr);
    if (ret != RCL_RET_OK) return ret;
  }

  return RCL_RET_OK;
}

rcl_ret_t HandleManager::CollectReadyHandles(rcl_wait_set_t* wait_set) {
  std::vector<rclnodejs::RclHandle*> ready_handles;
  CollectReadyHandlesByType(wait_set->subscriptions,
                            wait_set->size_of_subscriptions, subscriptions_,
                            &ready_handles);
  CollectReadyHandlesByType(wait_set->clients, wait_set->size_of_clients,
                            clients_, &ready_handles);
  CollectReadyHandlesByType(wait_set->services, wait_set->size_of_services,
                            services_, &ready_handles);
  CollectReadyHandlesByType(wait_set->timers, wait_set->size_of_timers, timers_,
                            &ready_handles);
  CollectReadyHandlesByType(wait_set->guard_conditions,
                            wait_set->size_of_guard_conditions,
                            guard_conditions_, &ready_handles);

  rcl_ret_t ret = CollectReadyActionHandles(wait_set, &ready_handles);
  if (!ready_handles.empty()) {
    ScopedReadWriteLock scoped_lock(&ready_handles_rwlock_,
                                    ScopedReadWriteLock::LockType::kWrite);
    ready_handles_ = std::move(ready_handles);
  }

  return ret;
}

rcl_ret_t HandleManager::GetEntityCounts(size_t* subscriptions_size,
                                         size_t* guard_conditions_size,
                                         size_t* timers_size,
                                         size_t* clients_size,
                                         size_t* services_size) {
  size_t num_subscriptions = 0u;
  size_t num_guard_conditions = 0u;
  size_t num_timers = 0u;
  size_t num_clients = 0u;
  size_t num_services = 0u;

  for (auto& action_client : action_clients_) {
    rcl_action_client_t* rcl_action_client =
        reinterpret_cast<rcl_action_client_t*>(action_client->ptr());
    rcl_ret_t ret = rcl_action_client_wait_set_get_num_entities(
        rcl_action_client, &num_subscriptions, &num_guard_conditions,
        &num_timers, &num_clients, &num_services);
    if (ret != RCL_RET_OK) {
      return ret;
    }

    *subscriptions_size += num_subscriptions;
    *guard_conditions_size += num_guard_conditions;
    *timers_size += num_timers;
    *clients_size += num_clients;
    *services_size += num_services;
  }

  for (auto& action_server : action_servers_) {
    rcl_action_server_t* rcl_action_server =
        reinterpret_cast<rcl_action_server_t*>(action_server->ptr());
    rcl_ret_t ret = rcl_action_server_wait_set_get_num_entities(
        rcl_action_server, &num_subscriptions, &num_guard_conditions,
        &num_timers, &num_clients, &num_services);
    if (ret != RCL_RET_OK) {
      return ret;
    }

    *subscriptions_size += num_subscriptions;
    *guard_conditions_size += num_guard_conditions;
    *timers_size += num_timers;
    *clients_size += num_clients;
    *services_size += num_services;
  }

  *subscriptions_size += subscription_count();
  *guard_conditions_size += guard_condition_count();
  *timers_size += timer_count();
  *clients_size += client_count();
  *services_size += service_count();

  return RCL_RET_OK;
}

std::vector<rclnodejs::RclHandle*> HandleManager::TakeReadyHandles() {
  ScopedReadWriteLock scoped_lock(&ready_handles_rwlock_,
                                  ScopedReadWriteLock::LockType::kWrite);
  std::vector<rclnodejs::RclHandle*> handles_to_be_taken(
      std::move(ready_handles_));
  return handles_to_be_taken;
}

uint32_t HandleManager::ready_handles_count() {
  ScopedReadWriteLock scoped_lock(&ready_handles_rwlock_,
                                  ScopedReadWriteLock::LockType::kRead);
  return ready_handles_.size();
}

uint32_t HandleManager::SynchronizeHandlesByType(
    const v8::Local<v8::Object>& typeObject,
    std::vector<rclnodejs::RclHandle*>* vec) {
  Nan::HandleScope scope;

  if (typeObject->IsArray()) {
    uint32_t length =
        Nan::To<uint32_t>(
            Nan::Get(typeObject, Nan::New("length").ToLocalChecked())
                .ToLocalChecked())
            .FromJust();

    for (uint32_t index = 0; index < length; index++) {
      v8::Local<v8::Object> obj =
          Nan::To<v8::Object>(Nan::Get(typeObject, index).ToLocalChecked())
              .ToLocalChecked();
      Nan::MaybeLocal<v8::Value> handle =
          Nan::Get(obj, Nan::New("_handle").ToLocalChecked());
      rclnodejs::RclHandle* rcl_handle =
          rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
              Nan::To<v8::Object>(handle.ToLocalChecked()).ToLocalChecked());
      vec->push_back(rcl_handle);
    }
  }
  return vec->size();
}

template <typename T>
void HandleManager::CollectReadyHandlesByType(
    const T** struct_ptr, size_t size,
    const std::vector<rclnodejs::RclHandle*>& handles,
    std::vector<rclnodejs::RclHandle*>* ready_handles) {
  for (size_t idx = 0; idx < size; ++idx) {
    if (struct_ptr[idx]) {
      for (auto& handle : handles) {
        if (struct_ptr[idx] == handle->ptr()) {
          ready_handles->push_back(handle);
        }
      }
    }
  }
}

rcl_ret_t HandleManager::CollectReadyActionHandles(
    rcl_wait_set_t* wait_set,
    std::vector<rclnodejs::RclHandle*>* ready_handles) {
  for (auto& action_client : action_clients_) {
    bool is_feedback_ready = false;
    bool is_status_ready = false;
    bool is_goal_response_ready = false;
    bool is_cancel_response_ready = false;
    bool is_result_response_ready = false;

    rcl_action_client_t* rcl_action_client =
        reinterpret_cast<rcl_action_client_t*>(action_client->ptr());
    rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
        wait_set, rcl_action_client, &is_feedback_ready, &is_status_ready,
        &is_goal_response_ready, &is_cancel_response_ready,
        &is_result_response_ready);
    if (ret != RCL_RET_OK) {
      return ret;
    }

    action_client->SetBoolProperty("isFeedbackReady", is_feedback_ready);
    action_client->SetBoolProperty("isStatusReady", is_status_ready);
    action_client->SetBoolProperty("isGoalResponseReady",
                                   is_goal_response_ready);
    action_client->SetBoolProperty("isCancelResponseReady",
                                   is_cancel_response_ready);
    action_client->SetBoolProperty("isResultResponseReady",
                                   is_result_response_ready);

    if (is_feedback_ready || is_status_ready || is_goal_response_ready ||
        is_cancel_response_ready || is_result_response_ready) {
      ready_handles->push_back(action_client);
    }
  }

  for (auto& action_server : action_servers_) {
    bool is_goal_request_ready = false;
    bool is_cancel_request_ready = false;
    bool is_result_request_ready = false;
    bool is_goal_expired = false;

    rcl_action_server_t* rcl_action_server =
        reinterpret_cast<rcl_action_server_t*>(action_server->ptr());
    rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
        wait_set, rcl_action_server, &is_goal_request_ready,
        &is_cancel_request_ready, &is_result_request_ready, &is_goal_expired);
    if (ret != RCL_RET_OK) {
      return ret;
    }

    action_server->SetBoolProperty("isGoalRequestReady", is_goal_request_ready);
    action_server->SetBoolProperty("isCancelRequestReady",
                                   is_cancel_request_ready);
    action_server->SetBoolProperty("isResultRequestReady",
                                   is_result_request_ready);
    action_server->SetBoolProperty("isGoalExpired", is_goal_expired);

    if (is_goal_request_ready || is_cancel_request_ready ||
        is_result_request_ready || is_goal_expired) {
      ready_handles->push_back(action_server);
    }
  }

  return RCL_RET_OK;
}

}  // namespace rclnodejs
