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

#include <vector>

#include "spdlog/spdlog.h"

namespace rclnodejs {

HandleManager::HandleManager() {
  is_synchronizing_.store(false);
  uv_mutex_init(&mutex_);
  uv_sem_init(&sem_, 0);
}

HandleManager::~HandleManager() {
  uv_mutex_destroy(&mutex_);
  uv_sem_destroy(&sem_);
}

void HandleManager::CollectHandles(const v8::Local<v8::Object> node) {
  is_synchronizing_.store(true);

  {
    ScopedMutex mutex(&mutex_);
    ClearHandles();
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

    CollectHandlesByType(
        Nan::To<v8::Object>(timers.ToLocalChecked()).ToLocalChecked(),
        &timers_);
    CollectHandlesByType(
        Nan::To<v8::Object>(subscriptions.ToLocalChecked()).ToLocalChecked(),
        &subscriptions_);
    CollectHandlesByType(
        Nan::To<v8::Object>(clients.ToLocalChecked()).ToLocalChecked(),
        &clients_);
    CollectHandlesByType(
        Nan::To<v8::Object>(services.ToLocalChecked()).ToLocalChecked(),
        &services_);
    CollectHandlesByType(
        Nan::To<v8::Object>(guard_conditions.ToLocalChecked()).ToLocalChecked(),
        &guard_conditions_);
  }

  is_synchronizing_.store(false);
  uv_sem_post(&sem_);

  SPDLOG_DEBUG(
      spdlog::get("rclnodejs"),
      "Add {0:d} timers, {1:d} subscriptions, {2:d} clients, " +
          "{3:d} services, {4:d} guards.",
      timers_.size(),
      subscriptions_.size(),
      clients_.size(),
      services_.size(),
      guard_conditions_.size());
}

bool HandleManager::AddHandlesToWaitSet(rcl_wait_set_t* wait_set) {
  for (auto& timer : timers_) {
    rcl_timer_t* rcl_timer = reinterpret_cast<rcl_timer_t*>(timer->ptr());
    if (rcl_wait_set_add_timer(wait_set, rcl_timer, nullptr) != RCL_RET_OK)
      return false;
  }
  for (auto& subscription : subscriptions_) {
    rcl_subscription_t* rcl_subscription  =
        reinterpret_cast<rcl_subscription_t*>(subscription->ptr());
    if (rcl_wait_set_add_subscription(wait_set, rcl_subscription, nullptr) !=
        RCL_RET_OK)
      return false;
  }
  for (auto& client : clients_) {
    rcl_client_t* rcl_client = reinterpret_cast<rcl_client_t*>(client->ptr());
    if (rcl_wait_set_add_client(wait_set, rcl_client, nullptr) != RCL_RET_OK)
      return false;
  }
  for (auto& service : services_) {
    rcl_service_t* rcl_service =
        reinterpret_cast<rcl_service_t*>(service->ptr());
    if (rcl_wait_set_add_service(wait_set, rcl_service, nullptr) != RCL_RET_OK)
      return false;
  }
  for (auto& guard_condition : guard_conditions_) {
    rcl_guard_condition_t* rcl_guard_condition =
        reinterpret_cast<rcl_guard_condition_t*>(guard_condition->ptr());
    if (rcl_wait_set_add_guard_condition(wait_set, rcl_guard_condition, nullptr)
        !=  RCL_RET_OK)
      return false;
  }

  return true;
}

void HandleManager::CollectReadyHandles(rcl_wait_set_t* wait_set) {
  ready_handles_.clear();

  CollectReadyHandlesByType(
    wait_set->subscriptions,
    wait_set->size_of_subscriptions,
    subscriptions_);
  CollectReadyHandlesByType(
    wait_set->clients,
    wait_set->size_of_clients,
    clients_);
  CollectReadyHandlesByType(
    wait_set->services,
    wait_set->size_of_services,
    services_);
  CollectReadyHandlesByType(
    wait_set->timers,
    wait_set->size_of_timers,
    timers_);
  CollectReadyHandlesByType(
    wait_set->guard_conditions,
    wait_set->size_of_guard_conditions,
    guard_conditions_);
}

void HandleManager::ClearHandles() {
  timers_.clear();
  clients_.clear();
  services_.clear();
  subscriptions_.clear();
  guard_conditions_.clear();
}

void HandleManager::CollectHandlesByType(
    const v8::Local<v8::Object>& typeObject,
    std::vector<rclnodejs::RclHandle*>* vec) {
  Nan::HandleScope scope;

  if (typeObject->IsArray()) {
    uint32_t length = Nan::To<uint32_t>(
        Nan::Get(typeObject, Nan::New("length").ToLocalChecked())
            .ToLocalChecked()).FromJust();

    for (uint32_t index = 0; index < length; index++) {
      v8::Local<v8::Object> obj =
          Nan::To<v8::Object>(typeObject->Get(index)).ToLocalChecked();
      Nan::MaybeLocal<v8::Value> handle =
          Nan::Get(obj, Nan::New("_handle").ToLocalChecked());
      rclnodejs::RclHandle* rcl_handle =
          rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
              Nan::To<v8::Object>(handle.ToLocalChecked()).ToLocalChecked());
      vec->push_back(rcl_handle);
    }
  }
}

template<typename T>
void HandleManager::CollectReadyHandlesByType(
    const T** struct_ptr,
    size_t size,
    const std::vector<rclnodejs::RclHandle*>& handles) {
  for (size_t idx = 0; idx < size; ++idx) {
    if (struct_ptr[idx]) {
      for (auto& handle : handles) {
        if (struct_ptr[idx] == handle->ptr()) {
          ready_handles_.push_back(handle);
        }
      }
    }
  }
}

}  // namespace rclnodejs
