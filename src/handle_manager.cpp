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

namespace rclnodejs {

struct ScopedMutex {
  explicit ScopedMutex(uv_mutex_t mutex) : mutex_(mutex) {
    uv_mutex_lock(&mutex_);
  }
  ~ScopedMutex() {
    uv_mutex_unlock(&mutex_);
  }
  uv_mutex_t mutex_;
};

HandleManager::HandleManager() {
  uv_mutex_init(&mutex_);
}

HandleManager::~HandleManager() {
  uv_mutex_destroy(&mutex_);
}

void HandleManager::CollectHandles(const v8::Local<v8::Object> node) {
  Nan::HandleScope scope;
  Nan::MaybeLocal<v8::Value> timers =
      Nan::Get(node, Nan::New("_timers").ToLocalChecked());
  Nan::MaybeLocal<v8::Value> subscriptions =
      Nan::Get(node, Nan::New("_subscriptions").ToLocalChecked());

  CollectHandlesByType(timers.ToLocalChecked()->ToObject(), &timers_);
  CollectHandlesByType(subscriptions.ToLocalChecked()->ToObject(),
                       &subscriptions_);
}

uint32_t HandleManager::SubscriptionsCount() {
  return subscriptions_.size();
}

uint32_t HandleManager::ServicesCount() {
  return services_.size();
}

uint32_t HandleManager::ClientsCount() {
  return clients_.size();
}

uint32_t HandleManager::TimersCount() {
  return timers_.size();
}

bool HandleManager::AddHandlesToWaitSet(rcl_wait_set_t* wait_set) {
  ScopedMutex scoped_mutex(mutex_);

  for (auto& timer : timers_) {
    if (rcl_wait_set_add_timer(wait_set, timer) != RCL_RET_OK)
      return false;
  }
  for (auto& subscription : subscriptions_) {
    if (rcl_wait_set_add_subscription(wait_set, subscription) != RCL_RET_OK)
      return false;
  }
  return true;
}

void HandleManager::ClearHandles() {
  ScopedMutex scoped_mutex(mutex_);
  timers_.clear();
  clients_.clear();
  services_.clear();
  subscriptions_.clear();
  guard_conditions_.clear();
}

template<typename T>
void HandleManager::CollectHandlesByType(
      const v8::Local<v8::Object>& typeObject, std::vector<const T*>* vec) {
  ScopedMutex scoped_mutex(mutex_);
  Nan::HandleScope scope;

  if (typeObject->IsArray()) {
    uint32_t length = Nan::Get(typeObject, Nan::New("length").ToLocalChecked()).
        ToLocalChecked()->Uint32Value();

    for (uint32_t index = 0; index < length; index++) {
        v8::Local<v8::Object> obj = typeObject->Get(index)->ToObject();
        Nan::MaybeLocal<v8::Value> handle =
            Nan::Get(obj, Nan::New("_handle").ToLocalChecked());
        rclnodejs::RclHandle* rcl_handle = rclnodejs::RclHandle::Unwrap<
            rclnodejs::RclHandle>(handle.ToLocalChecked()->ToObject());
        vec->push_back(reinterpret_cast<T*>(rcl_handle->GetPtr()));
    }
  }
}

}  // namespace rclnodejs
