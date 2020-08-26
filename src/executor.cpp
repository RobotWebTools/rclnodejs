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
#include "rcl_bindings.hpp"
#include "spdlog/spdlog.h"

namespace rclnodejs {

static std::exception_ptr g_exception_ptr = nullptr;

Executor::Executor(HandleManager* handle_manager, Delegate* delegate)
    : async_(nullptr),
      handle_manager_(handle_manager),
      delegate_(delegate),
      context_(nullptr) {
  running_.store(false);
}

Executor::~Executor() {
  // Note: don't free this->async_ in ctor
}

void Executor::Start(rcl_context_t* context, int32_t time_out) {
  if (!running_.load()) {
    async_ = new uv_async_t();
    context_ = context;
    time_out_ = time_out;

    uv_async_init(uv_default_loop(), async_, DoWork);
    async_->data = this;

    // Mark flag before creating thread
    // Make sure thread can run
    running_.store(true);
    uv_thread_create(&thread_, Executor::Run, this);
  }
}

void Executor::Stop() {
  if (running_.load()) {
    // Stop thread first, and then uv_close
    // Make sure async_ is not used anymore
    running_.store(false);
    uv_thread_join(&thread_);

    if (uv_is_active(reinterpret_cast<uv_handle_t*>(async_))) {
      static bool handle_closed = false;
      uv_close(reinterpret_cast<uv_handle_t*>(async_),
               [](uv_handle_t* async) -> void {
                 // Important Notice:
                 //  This might be called after Executor::~Executor()
                 //  Don't free Executor::async_ in Executor's dtor
                 delete async;
                 handle_closed = true;
               });
      while (!handle_closed)
        uv_run(uv_default_loop(), UV_RUN_ONCE);

      SPDLOG_DEBUG(spdlog::get("rclnodejs"), "Background thread stopped.");
    }
  }
}

void Executor::DoWork(uv_async_t* handle) {
  Executor* executor = reinterpret_cast<Executor*>(handle->data);
  if (executor->delegate_) {
    if (g_exception_ptr) {
      executor->delegate_->CatchException(g_exception_ptr);
      rcl_reset_error();
      g_exception_ptr = nullptr;
    }
    executor->delegate_->Execute();
  }
}

void Executor::Run(void* arg) {
  SPDLOG_DEBUG(spdlog::get("rclnodejs"), "Background thread started.");
  Executor* executor = reinterpret_cast<Executor*>(arg);
  HandleManager* handle_manager = executor->handle_manager_;

  try {
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    rcl_ret_t ret =
        rcl_wait_set_init(&wait_set, 0, 2, 0, 0, 0, 0, executor->context_,
                          rcl_get_default_allocator());
    if (ret != RCL_RET_OK) {
      throw std::runtime_error(std::string("Init waitset failed: ") +
                               rcl_get_error_string().str);
    }

    while (executor->running_.load()) {
      if (handle_manager->is_synchronizing())
        handle_manager->WaitForSynchronizing();

      {
        ScopedMutex mutex(handle_manager->mutex());
        if (handle_manager->is_empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }

        if (rcl_wait_set_resize(&wait_set, handle_manager->subscription_count(),
                                // TODO(minggang): support guard conditions
                                1u, handle_manager->timer_count(),
                                handle_manager->client_count(),
                                handle_manager->service_count(),
                                // TODO(minggang): support events.
                                0u) != RCL_RET_OK) {
          std::string error_message = std::string("Failed to resize: ") +
                                      std::string(rcl_get_error_string().str);
          throw std::runtime_error(error_message);
        }

        if (!handle_manager->AddHandlesToWaitSet(&wait_set)) {
          throw std::runtime_error("Couldn't fill waitset");
        }

        rcl_wait_set_add_guard_condition(&wait_set, g_sigint_gc, nullptr);

        int64_t time_out =
            executor->time_out() < 0 ? -1 : RCL_MS_TO_NS(executor->time_out());

        rcl_ret_t status = rcl_wait(&wait_set, time_out);
        if (status == RCL_RET_WAIT_SET_EMPTY) {
        } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
          throw std::runtime_error(std::string("rcl_wait() failed: ") +
                                   rcl_get_error_string().str);
        } else {
          if (wait_set.size_of_guard_conditions == 1 &&
              wait_set.guard_conditions[0]) {
            executor->running_.store(false);
          }
          if (!uv_is_closing(
                  reinterpret_cast<uv_handle_t*>(executor->async_))) {
            uv_async_send(executor->async_);
          }
        }

        if (rcl_wait_set_clear(&wait_set) != RCL_RET_OK) {
          std::string error_message =
              std::string("Failed to clear wait set: ") +
              std::string(rcl_get_error_string().str);
          throw std::runtime_error(error_message);
        }
      }
    }

    if (rcl_wait_set_fini(&wait_set) != RCL_RET_OK) {
      throw std::runtime_error(std::string("Failed to destroy guard waitset:") +
                               rcl_get_error_string().str);
    }
  } catch (...) {
    g_exception_ptr = std::current_exception();
    uv_async_send(executor->async_);
  }
}

}  // namespace rclnodejs
