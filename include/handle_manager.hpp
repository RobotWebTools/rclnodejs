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
#include <vector>

namespace rclnodejs {

class HandleManager {
 public:
  HandleManager();
  ~HandleManager();

  void CollectHandles(const v8::Local<v8::Object> node);

  uint32_t SubscriptionsCount();
  uint32_t ServicesCount();
  uint32_t ClientsCount();
  uint32_t TimersCount();

  bool AddHandlesToWaitSet(rcl_wait_set_t* wait_set);
  void ClearHandles();

 protected:
  template<typename T> void CollectHandlesByType(
      const v8::Local<v8::Object>& typeObject, std::vector<const T*>* vec);

 private:
  std::vector<const rcl_timer_t*> timers_;
  std::vector<const rcl_client_t*> clients_;
  std::vector<const rcl_service_t*> services_;
  std::vector<const rcl_subscription_t*> subscriptions_;
  std::vector<const rcl_guard_condition_t*> guard_conditions_;

  uv_mutex_t mutex_;
};

}  // namespace rclnodejs

#endif
