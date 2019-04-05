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

#include "rcl_handle.hpp"
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

    CollectHandlesByType(timers.ToLocalChecked()->ToObject(), &timers_);
    CollectHandlesByType(subscriptions.ToLocalChecked()->ToObject(),
                         &subscriptions_);
    CollectHandlesByType(clients.ToLocalChecked()->ToObject(), &clients_);
    CollectHandlesByType(services.ToLocalChecked()->ToObject(), &services_);
  }

  is_synchronizing_.store(false);
  uv_sem_post(&sem_);

  SPDLOG_DEBUG(
      spdlog::get("rclnodejs"),
      "Add {0:d} timers, {1:d} subscriptions, {2:d} clients, {3:d} services.",
      timers_.size(), subscriptions_.size(), clients_.size(), services_.size());
}

bool HandleManager::AddHandlesToWaitSet(rcl_wait_set_t* wait_set) {
  for (auto& timer : timers_) {
    if (rcl_wait_set_add_timer(wait_set, timer, nullptr) != RCL_RET_OK)
      return false;
  }
  for (auto& subscription : subscriptions_) {
    if (rcl_wait_set_add_subscription(wait_set, subscription, nullptr) !=
        RCL_RET_OK)
      return false;
  }
  for (auto& client : clients_) {
    if (rcl_wait_set_add_client(wait_set, client, nullptr) != RCL_RET_OK)
      return false;
  }
  for (auto& service : services_) {
    if (rcl_wait_set_add_service(wait_set, service, nullptr) != RCL_RET_OK)
      return false;
  }

  return true;
}

void HandleManager::ClearHandles() {
  timers_.clear();
  clients_.clear();
  services_.clear();
  subscriptions_.clear();
  guard_conditions_.clear();
}

template <typename T>
void HandleManager::CollectHandlesByType(
    const v8::Local<v8::Object>& typeObject,
    std::vector<const T*>* vec) {
  Nan::HandleScope scope;

  if (typeObject->IsArray()) {
    uint32_t length = Nan::Get(typeObject, Nan::New("length").ToLocalChecked())
                          .ToLocalChecked()
                          ->Uint32Value();

    for (uint32_t index = 0; index < length; index++) {
      v8::Local<v8::Object> obj = typeObject->Get(index)->ToObject();
      Nan::MaybeLocal<v8::Value> handle =
          Nan::Get(obj, Nan::New("_handle").ToLocalChecked());
      rclnodejs::RclHandle* rcl_handle =
          rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
              handle.ToLocalChecked()->ToObject());
      vec->push_back(reinterpret_cast<T*>(rcl_handle->ptr()));
    }
  }
}

}  // namespace rclnodejs
