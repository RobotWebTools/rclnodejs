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

#include <stdexcept>
#include <string>

#include "handle_manager.hpp"
#include "macros.hpp"
#include "rcl_bindings.hpp"

#ifdef WIN32
#define UNUSED
#else
#define UNUSED __attribute__((unused))
#endif

namespace rclnodejs {

static std::exception_ptr g_exception_ptr = nullptr;

struct RclResult {
  RclResult(rcl_ret_t rcl_ret, const std::string& msg)
      : ret(rcl_ret), error_msg(msg) {}

  rcl_ret_t ret;
  std::string error_msg;
};

Executor::Executor(HandleManager* handle_manager, Delegate* delegate)
    : async_(nullptr),
      main_thread_(uv_thread_self()),
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
    uv_thread_create(&background_thread_, Executor::Run, this);
  }
}

void Executor::SpinOnce(rcl_context_t* context, int32_t time_out) {
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(&wait_set, 0, 0, 0, 0, 0, 0, context,
                                    rcl_get_default_allocator());
  if (ret != RCL_RET_OK) Nan::ThrowError(rcl_get_error_string().str);

  RclResult wait_result = WaitForReadyCallbacks(&wait_set, time_out);

  if (wait_result.ret != RCL_RET_OK)
    Nan::ThrowError(wait_result.error_msg.c_str());

  if (handle_manager_->ready_handles_count() > 0) ExecuteReadyHandles();

  if (rcl_wait_set_fini(&wait_set) != RCL_RET_OK) {
    std::string error_message =
        std::string("Failed to destroy guard waitset:") +
        std::string(rcl_get_error_string().str);
    Nan::ThrowError(error_message.c_str());
  }
}

void Executor::Stop() {
  if (running_.load()) {
    // Stop thread first, and then uv_close
    // Make sure async_ is not used anymore
    running_.store(false);
    handle_manager_->StopWaitingHandles();
    uv_thread_join(&background_thread_);

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
      while (!handle_closed) uv_run(uv_default_loop(), UV_RUN_ONCE);

      RCLNODEJS_DEBUG("Background thread stopped.");
    }
  }
}

bool Executor::IsMainThread() {
  uv_thread_t this_thread = uv_thread_self();
  return uv_thread_equal(&main_thread_, &this_thread) != 0;
}

void Executor::DoWork(uv_async_t* handle) {
  Executor* executor = reinterpret_cast<Executor*>(handle->data);
  executor->ExecuteReadyHandles();
}

void Executor::Run(void* arg) {
  RCLNODEJS_DEBUG("Background thread started.");
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
      RclResult wait_result =
          executor->WaitForReadyCallbacks(&wait_set, executor->time_out());
      if (wait_result.ret != RCL_RET_OK)
        throw std::runtime_error(wait_result.error_msg);

      if (!uv_is_closing(reinterpret_cast<uv_handle_t*>(executor->async_)) &&
          handle_manager->ready_handles_count() > 0) {
        uv_async_send(executor->async_);
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

RclResult Executor::WaitForReadyCallbacks(rcl_wait_set_t* wait_set,
                                          int32_t time_out) {
  // Wait the handles on the background thread if there is none.
  if (handle_manager_->sum() == 0 && !IsMainThread())
    handle_manager_->WaitForHandles();

  if (handle_manager_->is_synchronizing())
    handle_manager_->WaitForSynchronizing();

  {
    ScopedReadWriteLock read_lock(handle_manager_->handle_rwlock(),
                                  ScopedReadWriteLock::LockType::kRead);

    size_t num_subscriptions = 0u;
    size_t num_guard_conditions = 0u;
    size_t num_timers = 0u;
    size_t num_clients = 0u;
    size_t num_services = 0u;

    rcl_ret_t get_entity_ret = handle_manager_->GetEntityCounts(
        &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients,
        &num_services);
    if (get_entity_ret != RCL_RET_OK) {
      std::string error_message = std::string("Failed to get entity counts: ") +
                                  std::string(rcl_get_error_string().str);
      return RclResult(get_entity_ret, error_message);
    }

    rcl_ret_t resize_ret =
        rcl_wait_set_resize(wait_set, num_subscriptions, num_guard_conditions,
                            num_timers, num_clients, num_services,
                            // TODO(minggang): support events.
                            0u);
    if (resize_ret != RCL_RET_OK) {
      std::string error_message = std::string("Failed to resize: ") +
                                  std::string(rcl_get_error_string().str);
      return RclResult(resize_ret, error_message);
    }

    rcl_ret_t add_wait_set_ret = handle_manager_->AddHandlesToWaitSet(wait_set);
    if (add_wait_set_ret != RCL_RET_OK) {
      std::string error_message =
          std::string("Couldn't fill waitset") + rcl_get_error_string().str;
      return RclResult(add_wait_set_ret, error_message);
    }

    time_out = time_out < 0 ? -1 : RCL_MS_TO_NS(time_out);

    rcl_ret_t wait_ret = rcl_wait(wait_set, time_out);

    if (wait_ret == RCL_RET_WAIT_SET_EMPTY) {
    } else if (wait_ret != RCL_RET_OK && wait_ret != RCL_RET_TIMEOUT) {
      std::string error_message =
          std::string("rcl_wait() failed: ") + rcl_get_error_string().str;
      return RclResult(wait_ret, error_message);
    } else {
      // If ready_handles_count() returns a value which is greater than 0, it
      // means that the previous ready handles haven't been taken. So stop here
      // and return.
      if (handle_manager_->ready_handles_count() > 0)
        return RclResult(RCL_RET_OK, "" /* msg */);

      rcl_ret_t collect_handles_ret =
          handle_manager_->CollectReadyHandles(wait_set);
      if (collect_handles_ret != RCL_RET_OK) {
        std::string error_message =
            std::string("Failed to collect ready handles: ") +
            std::string(rcl_get_error_string().str);
        return RclResult(wait_ret, error_message);
      }
    }
  }

  rcl_ret_t set_clear_ret = rcl_wait_set_clear(wait_set);
  if (set_clear_ret != RCL_RET_OK) {
    std::string error_message = std::string("Failed to clear wait set: ") +
                                std::string(rcl_get_error_string().str);
    return RclResult(set_clear_ret, error_message);
  }

  return RclResult(RCL_RET_OK, "" /* msg */);
}

void Executor::ExecuteReadyHandles() {
  if (delegate_) {
    if (g_exception_ptr) {
      delegate_->CatchException(g_exception_ptr);
      rcl_reset_error();
      g_exception_ptr = nullptr;
      return;
    }

    delegate_->Execute(handle_manager_->TakeReadyHandles());
  }
}

}  // namespace rclnodejs
