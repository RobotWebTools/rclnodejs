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

#include "executor.h"

#include <rcl/error_handling.h>

#include <stdexcept>
#include <string>

#include "handle_manager.h"
#include "macros.h"
#include "rcl_bindings.h"

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

Executor::Executor(Napi::Env env, HandleManager* handle_manager,
                   Delegate* delegate)
    : async_(nullptr),
      main_thread_(uv_thread_self()),
      handle_manager_(handle_manager),
      delegate_(delegate),
      context_(nullptr),
      interrupt_guard_condition_(rcl_get_zero_initialized_guard_condition()),
      env_(env),
      state_(ExecutorState::Stopped),
      work_pending_(false) {}

Executor::~Executor() {
  // Note: don't free this->async_ in ctor
}

void Executor::Start(rcl_context_t* context, int32_t time_out) {
  if (state_.load() != ExecutorState::Stopped) {
    Napi::Error::New(env_, "Cannot start executor until it is stopped")
        .ThrowAsJavaScriptException();
    return;
  }
  state_.store(ExecutorState::Starting);
  context_ = context;
  time_out_ = time_out;

  rcl_ret_t ret =
      rcl_guard_condition_init(&interrupt_guard_condition_, context_,
                               rcl_guard_condition_get_default_options());
  if (ret != RCL_RET_OK) {
    std::string error_message =
        std::string("Failed to initialize executor guard condition: ") +
        rcl_get_error_string().str;
    rcl_reset_error();
    state_.store(ExecutorState::Stopped);
    Napi::Error::New(env_, error_message).ThrowAsJavaScriptException();
    return;
  }

  async_ = new uv_async_t();
  int uv_ret = uv_async_init(uv_default_loop(), async_, DoWork);
  if (uv_ret == 0) {
    async_->data = this;
    {
      // Run() takes this lock before checking the published startup state.
      std::lock_guard<std::mutex> lock(work_done_mutex_);
      work_pending_ = false;
      uv_ret = uv_thread_create(&background_thread_, Executor::Run, this);
      if (uv_ret == 0) {
        state_.store(ExecutorState::Running);
        return;
      }
    }
  } else {
    delete async_;
    async_ = nullptr;
  }

  std::string error_message =
      std::string("Failed to start executor: ") + uv_strerror(uv_ret);
  const std::string cleanup_error = ReleaseResources();
  if (!cleanup_error.empty()) {
    error_message += "; " + cleanup_error;
  }
  Napi::Error::New(env_, error_message).ThrowAsJavaScriptException();
}

void Executor::SpinOnce(rcl_context_t* context, int32_t time_out) {
  if (state_.load() != ExecutorState::Stopped) {
    Napi::Error::New(env_, "Cannot spin once until executor is stopped")
        .ThrowAsJavaScriptException();
    return;
  }
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(&wait_set, 0, 0, 0, 0, 0, 0, context,
                                    rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    Napi::Error::New(env_, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    return;
  }

  RclResult wait_result = WaitForReadyCallbacks(&wait_set, time_out);

  if (wait_result.ret != RCL_RET_OK) {
    Napi::Error::New(env_, wait_result.error_msg.c_str())
        .ThrowAsJavaScriptException();
    return;
  }

  if (handle_manager_->ready_handles_count() > 0) ExecuteReadyHandles();

  if (rcl_wait_set_fini(&wait_set) != RCL_RET_OK) {
    std::string error_message =
        std::string("Failed to destroy guard waitset:") +
        std::string(rcl_get_error_string().str);
    Napi::Error::New(env_, error_message.c_str()).ThrowAsJavaScriptException();
  }
}

void Executor::Stop() {
  switch (state_.load()) {
    case ExecutorState::Stopped:
    case ExecutorState::ReleasingResources:
      return;
    case ExecutorState::Starting:
      Napi::Error::New(env_, "Cannot stop executor during startup")
          .ThrowAsJavaScriptException();
      return;
    case ExecutorState::Running:
    case ExecutorState::ThreadJoinPending:
      break;
  }

  {
    std::lock_guard<std::mutex> lock(work_done_mutex_);
    state_.store(ExecutorState::ThreadJoinPending);
  }
  work_done_cv_.notify_all();
  handle_manager_->StopWaitingHandles();
  rcl_ret_t ret = rcl_trigger_guard_condition(&interrupt_guard_condition_);
  if (ret != RCL_RET_OK) {
    std::string error_message = std::string("Failed to interrupt executor: ") +
                                rcl_get_error_string().str;
    rcl_reset_error();
    Napi::Error::New(env_, error_message).ThrowAsJavaScriptException();
    return;
  }
  int uv_ret = uv_thread_join(&background_thread_);
  if (uv_ret != 0) {
    Napi::Error::New(env_, std::string("Failed to join executor thread: ") +
                               uv_strerror(uv_ret))
        .ThrowAsJavaScriptException();
    return;
  }

  const std::string error_message = ReleaseResources();
  if (!error_message.empty()) {
    Napi::Error::New(env_, error_message).ThrowAsJavaScriptException();
  }
}

std::string Executor::ReleaseResources() {
  state_.store(ExecutorState::ReleasingResources);
  handle_manager_->TakeReadyHandles();

  std::string error_message;
  if (rcl_guard_condition_fini(&interrupt_guard_condition_) != RCL_RET_OK) {
    error_message =
        std::string("Failed to finalize executor guard condition: ") +
        rcl_get_error_string().str;
    rcl_reset_error();
  }

  uv_async_t* async = async_;
  async_ = nullptr;
  if (async) {
    // Do not pump the loop during teardown; the close callback owns this
    // handle.
    async->data = nullptr;
    uv_close(reinterpret_cast<uv_handle_t*>(async), [](uv_handle_t* handle) {
      delete reinterpret_cast<uv_async_t*>(handle);
    });
  }

  state_.store(ExecutorState::Stopped);
  return error_message;
}

bool Executor::IsMainThread() {
  uv_thread_t this_thread = uv_thread_self();
  return uv_thread_equal(&main_thread_, &this_thread) != 0;
}

void Executor::DoWork(uv_async_t* handle) {
  Executor* executor = reinterpret_cast<Executor*>(handle->data);

  // RAII guard: always clear work_pending_ and notify the background thread,
  // even if ExecuteReadyHandles() throws (e.g. from N-API callbacks).
  // Without this, the background thread would block forever on work_done_cv_.
  struct WorkDoneGuard {
    Executor* exec;
    ~WorkDoneGuard() {
      {
        std::lock_guard<std::mutex> lock(exec->work_done_mutex_);
        exec->work_pending_ = false;
      }
      exec->work_done_cv_.notify_one();
    }
  } guard{executor};

  executor->ExecuteReadyHandles();
}

void Executor::Run(void* arg) {
  RCLNODEJS_DEBUG("Background thread started.");
  Executor* executor = reinterpret_cast<Executor*>(arg);
  HandleManager* handle_manager = executor->handle_manager_;

  {
    std::lock_guard<std::mutex> lock(executor->work_done_mutex_);
    if (!executor->IsRunning()) return;
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  bool wait_set_initialized = false;
  std::string error_message;
  try {
    rcl_ret_t ret =
        rcl_wait_set_init(&wait_set, 0, 2, 0, 0, 0, 0, executor->context_,
                          rcl_get_default_allocator());
    if (ret != RCL_RET_OK) {
      throw std::runtime_error(std::string("Init waitset failed: ") +
                               rcl_get_error_string().str);
    }
    wait_set_initialized = true;

    while (executor->IsRunning()) {
      RclResult wait_result =
          executor->WaitForReadyCallbacks(&wait_set, executor->time_out());
      if (wait_result.ret != RCL_RET_OK)
        throw std::runtime_error(wait_result.error_msg);

      if (!executor->IsRunning()) break;

      if (!uv_is_closing(reinterpret_cast<uv_handle_t*>(executor->async_)) &&
          handle_manager->ready_handles_count() > 0) {
        // Tell the main thread there is work to do, then wait for it to
        // finish before re-entering rcl_wait.  This prevents a data race
        // where the background thread holds subscriptions in the wait set
        // while the main thread modifies their state (e.g. content filter).
        {
          std::lock_guard<std::mutex> lock(executor->work_done_mutex_);
          executor->work_pending_ = true;
        }
        uv_async_send(executor->async_);

        // Wait until DoWork() signals completion.
        {
          std::unique_lock<std::mutex> lock(executor->work_done_mutex_);
          executor->work_done_cv_.wait(lock, [executor] {
            return !executor->work_pending_ || !executor->IsRunning();
          });
        }
      }
    }
  } catch (const std::exception& error) {
    error_message = error.what();
  }
  if (wait_set_initialized && rcl_wait_set_fini(&wait_set) != RCL_RET_OK) {
    if (!error_message.empty()) error_message += "; ";
    error_message += std::string("Failed to destroy executor wait set: ") +
                     rcl_get_error_string().str;
  }
  if (!error_message.empty()) {
    {
      std::lock_guard<std::mutex> lock(executor->work_done_mutex_);
      executor->state_.store(ExecutorState::ThreadJoinPending);
    }
    g_exception_ptr =
        std::make_exception_ptr(std::runtime_error(error_message));
    uv_async_send(executor->async_);
  }
}

RclResult Executor::WaitForReadyCallbacks(rcl_wait_set_t* wait_set,
                                          int32_t time_out) {
  const bool is_background_thread = !IsMainThread();
  // Wake tokens can outlive the handles that produced them.
  while (is_background_thread && IsRunning() && handle_manager_->sum() == 0)
    handle_manager_->WaitForHandles();

  if (is_background_thread && !IsRunning()) return RclResult(RCL_RET_OK, "");

  if (handle_manager_->is_synchronizing())
    handle_manager_->WaitForSynchronizing();

  {
    ScopedReadWriteLock read_lock(handle_manager_->handle_rwlock(),
                                  ScopedReadWriteLock::LockType::kRead);

    // The last entity may disappear between the semaphore check and this lock.
    if (is_background_thread && (!IsRunning() || handle_manager_->sum() == 0)) {
      return RclResult(RCL_RET_OK, "");
    }

    size_t num_subscriptions = 0u;
    size_t num_guard_conditions = is_background_thread ? 1u : 0u;
    size_t num_timers = 0u;
    size_t num_clients = 0u;
    size_t num_services = 0u;
    size_t num_events = 0u;

    rcl_ret_t get_entity_ret = handle_manager_->GetEntityCounts(
        &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients,
        &num_services, &num_events);
    if (get_entity_ret != RCL_RET_OK) {
      std::string error_message = std::string("Failed to get entity counts: ") +
                                  std::string(rcl_get_error_string().str);
      return RclResult(get_entity_ret, error_message);
    }

    rcl_ret_t resize_ret =
        rcl_wait_set_resize(wait_set, num_subscriptions, num_guard_conditions,
                            num_timers, num_clients, num_services, num_events);
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

    if (is_background_thread) {
      rcl_ret_t guard_ret = rcl_wait_set_add_guard_condition(
          wait_set, &interrupt_guard_condition_, nullptr);
      if (guard_ret != RCL_RET_OK) {
        return RclResult(
            guard_ret, std::string("Failed to add executor guard condition: ") +
                           rcl_get_error_string().str);
      }
    }

    const int64_t timeout_ns =
        time_out < 0 ? -1 : RCL_MS_TO_NS(static_cast<int64_t>(time_out));
    rcl_ret_t wait_ret = rcl_wait(wait_set, timeout_ns);

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
