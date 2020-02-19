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
#include <rcl_action/rcl_action.h>

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
    Nan::MaybeLocal<v8::Value> action_clients =
        Nan::Get(node, Nan::New("_actionClients").ToLocalChecked());
    Nan::MaybeLocal<v8::Value> action_servers =
        Nan::Get(node, Nan::New("_actionServers").ToLocalChecked());

    CollectHandlesByType(timers.ToLocalChecked()->ToObject(), &timers_);
    CollectHandlesByType(subscriptions.ToLocalChecked()->ToObject(),
                         &subscriptions_);
    CollectHandlesByType(clients.ToLocalChecked()->ToObject(), &clients_);
    CollectHandlesByType(services.ToLocalChecked()->ToObject(), &services_);
    CollectHandlesByType(guard_conditions.ToLocalChecked()->ToObject(),
                         &guard_conditions_);
    CollectHandlesByType(action_clients.ToLocalChecked()->ToObject(),
                         &action_clients_);
    CollectHandlesByType(action_servers.ToLocalChecked()->ToObject(),
                         &action_servers_);
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
  for (auto& action_client : action_clients_) {
    rcl_action_client_t* rcl_action_client =
        reinterpret_cast<rcl_action_client_t*>(action_client->ptr());
    if (rcl_action_wait_set_add_action_client(wait_set, rcl_action_client,
        nullptr, nullptr) !=  RCL_RET_OK)
      return false;
  }
  for (auto& action_server : action_servers_) {
    rcl_action_server_t* rcl_action_server =
        reinterpret_cast<rcl_action_server_t*>(action_server->ptr());
    if (rcl_action_wait_set_add_action_server(wait_set, rcl_action_server,
        nullptr) != RCL_RET_OK)
      return false;
  }

  return true;
}

bool HandleManager::CollectReadyHandles(rcl_wait_set_t* wait_set) {
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
  
  return CollectReadyActionHandles(wait_set);
}

bool HandleManager::GetEntityCounts(
    size_t *subscriptions_size,
    size_t *guard_conditions_size,
    size_t *timers_size,
    size_t *clients_size,
    size_t *services_size) {
  size_t num_subscriptions = 0u;
  size_t num_guard_conditions = 0u;
  size_t num_timers = 0u;
  size_t num_clients = 0u;
  size_t num_services = 0u;

  for (auto& action_client : action_clients_) {
    rcl_action_client_t* rcl_action_client =
        reinterpret_cast<rcl_action_client_t*>(action_client->ptr());
    rcl_ret_t ret = rcl_action_client_wait_set_get_num_entities(
        rcl_action_client,
        &num_subscriptions,
        &num_guard_conditions,
        &num_timers,
        &num_clients,
        &num_services);
    if (ret != RCL_RET_OK) {
      return false;
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
        rcl_action_server,
        &num_subscriptions,
        &num_guard_conditions,
        &num_timers,
        &num_clients,
        &num_services);
    if (ret != RCL_RET_OK) {
      return false;
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

  return true;
}

void HandleManager::ClearHandles() {
  timers_.clear();
  clients_.clear();
  services_.clear();
  subscriptions_.clear();
  guard_conditions_.clear();
  action_clients_.clear();
  action_servers_.clear();
}

void HandleManager::CollectHandlesByType(
    const v8::Local<v8::Object>& typeObject,
    std::vector<rclnodejs::RclHandle*>* vec) {
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

bool HandleManager::CollectReadyActionHandles(rcl_wait_set_t* wait_set) {
  for (auto& action_client : action_clients_) {
    bool is_feedback_ready = false;
    bool is_status_ready = false;
    bool is_goal_response_ready = false;
    bool is_cancel_response_ready = false;
    bool is_result_response_ready = false;

    rcl_action_client_t* rcl_action_client =
        reinterpret_cast<rcl_action_client_t*>(action_client->ptr());
    rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
        wait_set,
        rcl_action_client,
        &is_feedback_ready,
        &is_status_ready,
        &is_goal_response_ready,
        &is_cancel_response_ready,
        &is_result_response_ready);
    if (ret != RCL_RET_OK) {
      return false;
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
      ready_handles_.push_back(action_client);
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
        wait_set,
        rcl_action_server,
        &is_goal_request_ready,
        &is_cancel_request_ready,
        &is_result_request_ready,
        &is_goal_expired);
    if (ret != RCL_RET_OK) {
      return false;
    }

    action_server->SetBoolProperty("isGoalRequestReady", is_goal_request_ready);
    action_server->SetBoolProperty("isCancelRequestReady",
                                   is_cancel_request_ready);
    action_server->SetBoolProperty("isResultRequestReady",
                                   is_result_request_ready);
    action_server->SetBoolProperty("isGoalExpired", is_goal_expired);

    if (is_goal_request_ready || is_cancel_request_ready ||
        is_result_request_ready || is_goal_expired) {
      ready_handles_.push_back(action_server);
    }
  }

  return true;
}

}  // namespace rclnodejs
