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

#include "executor.hpp"

#include <rcl/error_handling.h>
#include <rcl/wait.h>
#include <stdexcept>
#include <string>

#include "handle_manager.hpp"

namespace rclnodejs {

static std::exception_ptr g_exception_ptr = nullptr;

Executor::Executor(HandleManager* handle_manager, Delegate* delegate)
    : mainthread_loop_(uv_default_loop()),
      handle_manager_(handle_manager),
      delegate_(delegate) {
  async_ = reinterpret_cast<uv_async_t*>(malloc(sizeof(uv_async_t)));
  async_->data = this;
}

Executor::~Executor() {
  free(async_);
}

void Executor::Start() {
  if (!running_.load()) {
    uv_async_init(mainthread_loop_, async_, DoWork);
    uv_thread_create(&thread_, Executor::Run, this);
    running_.store(true);
  }
}

void Executor::Stop() {
  if (running_.load()) {
    running_.store(false);
    uv_close(reinterpret_cast<uv_handle_t*>(async_), nullptr);
    uv_thread_join(&thread_);
  }
}

void Executor::DoWork(uv_async_t* handle) {
  Executor*  executor = reinterpret_cast<Executor*>(handle->data);
  if (executor->delegate_) {
    if (g_exception_ptr) {
      executor->delegate_->CatchException(g_exception_ptr);
      g_exception_ptr = nullptr;
    }
    executor->delegate_->Execute();
  }
}

void Executor::Run(void* arg) {
  Executor* executor = reinterpret_cast<Executor*>(arg);
  HandleManager* handle_manager = executor->handle_manager_;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(&wait_set, 0, 2, 0, 0, 0,
      rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(std::string("Init waitset failed: ") +
        rcl_get_error_string_safe());
  }

  try {
    while (executor->running_.load()) {
      if (rcl_wait_set_resize_subscriptions(
          &wait_set, handle_manager->SubscriptionsCount()) != RCL_RET_OK) {
        throw std::runtime_error(std::string(
            "Couldn't resize the number of subscriptions in waitset : ") +
            rcl_get_error_string_safe());
      }

      if (rcl_wait_set_resize_services(
          &wait_set, handle_manager->ServicesCount()) != RCL_RET_OK) {
        throw std::runtime_error(std::string(
            "Couldn't resize the number of services in waitset : ") +
            rcl_get_error_string_safe());
      }

      if (rcl_wait_set_resize_clients(
          &wait_set, handle_manager->ClientsCount()) != RCL_RET_OK) {
        throw std::runtime_error(std::string(
            "Couldn't resize the number of clients in waitset : ") +
            rcl_get_error_string_safe());
      }

        if (rcl_wait_set_resize_timers(
            &wait_set, handle_manager->TimersCount()) != RCL_RET_OK) {
        throw std::runtime_error(std::string(
            "Couldn't resize the number of timers in waitset : ") +
            rcl_get_error_string_safe());
      }

      if (!handle_manager->AddHandlesToWaitSet(&wait_set)) {
        throw std::runtime_error("Couldn't fill waitset");
      }

      rcl_ret_t status =
          rcl_wait(&wait_set, RCL_MS_TO_NS(10));
      if (status == RCL_RET_WAIT_SET_EMPTY) {
      } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
        throw std::runtime_error(std::string("rcl_wait() failed: ") +
            rcl_get_error_string_safe());
      } else {
        uv_async_send(executor->async_);
      }

      if (rcl_wait_set_clear_subscriptions(&wait_set) != RCL_RET_OK) {
        throw std::runtime_error("Couldn't clear subscriptions from waitset");
      }

      if (rcl_wait_set_clear_services(&wait_set) != RCL_RET_OK) {
        throw std::runtime_error("Couldn't clear servicess from waitset");
      }

      if (rcl_wait_set_clear_clients(&wait_set) != RCL_RET_OK) {
        throw std::runtime_error("Couldn't clear clients from waitset");
      }

      if (rcl_wait_set_clear_guard_conditions(&wait_set) != RCL_RET_OK) {
        throw std::runtime_error(
            "Couldn't clear guard conditions from waitset");
      }

      if (rcl_wait_set_clear_timers(&wait_set) != RCL_RET_OK) {
        throw std::runtime_error("Couldn't clear timers from waitset");
      }
    }

    if (rcl_wait_set_fini(&wait_set) != RCL_RET_OK) {
      throw std::runtime_error(std::string(
          "Failed to destroy guard waitset:") + rcl_get_error_string_safe());
    }
  } catch(...) {
    g_exception_ptr = std::current_exception();
    uv_async_send(executor->async_);
  }
}

}  // namespace rclnodejs
