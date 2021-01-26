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

#ifndef RCLNODEJS_EXECUTOR_HPP_
#define RCLNODEJS_EXECUTOR_HPP_

#include <rcl/wait.h>
#include <uv.h>

#include <atomic>
#include <exception>
#include <vector>

#include "rcl_handle.hpp"

struct rcl_context_t;

namespace rclnodejs {

class HandleManager;
struct RclResult;

class Executor {
 public:
  class Delegate {
   public:
    virtual void Execute(const std::vector<rclnodejs::RclHandle*>& handles) = 0;
    virtual void CatchException(std::exception_ptr e_ptr) = 0;
  };

  Executor(HandleManager* handle_manager, Delegate* delegate);
  ~Executor();

  void Start(rcl_context_t* context, int32_t time_out);
  void Stop();
  void SpinOnce(rcl_context_t* context, int32_t time_out);
  int32_t time_out() { return time_out_; }
  bool IsMainThread();

  static void DoWork(uv_async_t* handle);
  static void Run(void* arg);

 private:
  // Returns RclResult object which indicates the final result.
  RclResult WaitForReadyCallbacks(rcl_wait_set_t* wait_set, int32_t time_out);

  // Calls the callback of the ready handle.
  void ExecuteReadyHandles();

  uv_async_t* async_;

  // The v8 main thread.
  uv_thread_t main_thread_;

  // Sub thread used to query the ready handles.
  uv_thread_t background_thread_;

  HandleManager* handle_manager_;
  Delegate* delegate_;
  rcl_context_t* context_;
  int32_t time_out_;

  std::atomic_bool running_;
};

}  // namespace rclnodejs

#endif
