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

#include <uv.h>

#include <atomic>
#include <exception>

namespace rclnodejs {

class HandleManager;

class Executor {
 public:
  class Delegate {
   public:
    virtual void Execute() = 0;
    virtual void CatchException(std::exception_ptr e_ptr) = 0;
  };

  Executor(HandleManager* handle_manager, Delegate* delegate);
  ~Executor();

  void Start();
  void Stop();

  static void DoWork(uv_async_t* handle);
  static void Run(void* arg);

 private:
  uv_async_t* async_;
  uv_thread_t thread_;

  HandleManager* handle_manager_;
  Delegate* delegate_;

  std::atomic_bool running_;
};

}  // namespace rclnodejs

#endif
