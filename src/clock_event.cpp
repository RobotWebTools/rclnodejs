// Copyright (c) 2025, The Robot Web Tools Contributors
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

#include "clock_event.hpp"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/types.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "macros.h"
#include "rcl_handle.h"

namespace rclnodejs {

template <typename ClockType>
void ClockEvent::wait_until(rcl_clock_t* clock, rcl_time_point_t until) {
  // Synchronize because clock epochs might differ
  rcl_time_point_value_t now_value;
  rcl_ret_t ret = rcl_clock_get_now(clock, &now_value);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to get current time");
  }
  rcl_time_point_t rcl_entry;
  rcl_entry.nanoseconds = now_value;
  rcl_entry.clock_type = clock->type;

  const typename ClockType::time_point chrono_entry = ClockType::now();

  rcl_duration_t delta_t;
  ret = rcl_difference_times(&rcl_entry, &until, &delta_t);

  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to subtract times");
  }

  // Cast because system clock resolution is too big for nanoseconds on Windows
  // & OSX
  const typename ClockType::time_point chrono_until =
      chrono_entry + std::chrono::duration_cast<typename ClockType::duration>(
                         std::chrono::nanoseconds(delta_t.nanoseconds));

  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_until(lock, chrono_until, [this]() { return state_; });
}

void ClockEvent::wait_until_ros(rcl_clock_t* clock, rcl_time_point_t until) {
  bool is_enabled;
  rcl_ret_t ret = rcl_is_enabled_ros_time_override(clock, &is_enabled);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to check if ros time override is enabled");
  }

  // Check if ROS time is enabled in C++ to avoid TOCTTOU with TimeSource
  if (is_enabled) {
    std::unique_lock<std::mutex> lock(mutex_);
    // Caller must have setup a time jump callback to wake this event
    cv_.wait(lock, [this]() { return state_; });
  } else {
    // ROS time not enabled is system time
    wait_until<std::chrono::system_clock>(clock, until);
  }
}

bool ClockEvent::is_set() {
  std::unique_lock<std::mutex> lock(mutex_);
  return state_;
}

void ClockEvent::set() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    state_ = true;
  }
  cv_.notify_all();
}

void ClockEvent::clear() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    state_ = false;
  }
  cv_.notify_all();
}

// Explicit instantiation
template void ClockEvent::wait_until<std::chrono::steady_clock>(
    rcl_clock_t* clock, rcl_time_point_t until);
template void ClockEvent::wait_until<std::chrono::system_clock>(
    rcl_clock_t* clock, rcl_time_point_t until);

enum class WaitType { Steady, System, Ros };

class ClockEventWaitWorker : public Napi::AsyncWorker {
 public:
  ClockEventWaitWorker(Napi::Env env, ClockEvent* event, rcl_clock_t* clock,
                       int64_t until, WaitType type)
      : Napi::AsyncWorker(env),
        event_(event),
        clock_(clock),
        until_(until),
        type_(type),
        deferred_(Napi::Promise::Deferred::New(env)) {}

  ~ClockEventWaitWorker() {}

  void Execute() override {
    try {
      rcl_time_point_t until_time_point;
      until_time_point.nanoseconds = until_;
      until_time_point.clock_type = clock_->type;

      switch (type_) {
        case WaitType::Ros:
          event_->wait_until_ros(clock_, until_time_point);
          break;
        case WaitType::Steady:
          event_->wait_until<std::chrono::steady_clock>(clock_,
                                                        until_time_point);
          break;
        case WaitType::System:
          event_->wait_until<std::chrono::system_clock>(clock_,
                                                        until_time_point);
          break;
      }
    } catch (const std::exception& e) {
      SetError(e.what());
    }
  }

  void OnOK() override { deferred_.Resolve(Env().Undefined()); }

  void OnError(const Napi::Error& e) override { deferred_.Reject(e.Value()); }

  Napi::Promise Promise() { return deferred_.Promise(); }

 private:
  ClockEvent* event_;
  rcl_clock_t* clock_;
  int64_t until_;
  WaitType type_;
  Napi::Promise::Deferred deferred_;
};

Napi::Value CreateClockEvent(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  ClockEvent* event = new ClockEvent();
  return RclHandle::NewInstance(env, event, nullptr, [](void* ptr) {
    delete static_cast<ClockEvent*>(ptr);
  });
}

Napi::Value ClockEventWaitUntilSteady(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* event_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  ClockEvent* event = static_cast<ClockEvent*>(event_handle->ptr());

  RclHandle* clock_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_clock_t* clock = static_cast<rcl_clock_t*>(clock_handle->ptr());

  bool lossless;
  int64_t until = info[2].As<Napi::BigInt>().Int64Value(&lossless);
  if (!lossless) {
    Napi::TypeError::New(env, "until value lost precision during conversion")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  auto worker =
      new ClockEventWaitWorker(env, event, clock, until, WaitType::Steady);
  worker->Queue();
  return worker->Promise();
}

Napi::Value ClockEventWaitUntilSystem(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* event_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  ClockEvent* event = static_cast<ClockEvent*>(event_handle->ptr());

  RclHandle* clock_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_clock_t* clock = static_cast<rcl_clock_t*>(clock_handle->ptr());

  bool lossless;
  int64_t until = info[2].As<Napi::BigInt>().Int64Value(&lossless);
  if (!lossless) {
    Napi::TypeError::New(env, "until value lost precision during conversion")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  auto worker =
      new ClockEventWaitWorker(env, event, clock, until, WaitType::System);
  worker->Queue();
  return worker->Promise();
}

Napi::Value ClockEventWaitUntilRos(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* event_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  ClockEvent* event = static_cast<ClockEvent*>(event_handle->ptr());

  RclHandle* clock_handle = RclHandle::Unwrap(info[1].As<Napi::Object>());
  rcl_clock_t* clock = static_cast<rcl_clock_t*>(clock_handle->ptr());

  bool lossless;
  int64_t until = info[2].As<Napi::BigInt>().Int64Value(&lossless);
  if (!lossless) {
    Napi::TypeError::New(env, "until value lost precision during conversion")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  auto worker =
      new ClockEventWaitWorker(env, event, clock, until, WaitType::Ros);
  worker->Queue();
  return worker->Promise();
}

Napi::Value ClockEventIsSet(const Napi::CallbackInfo& info) {
  RclHandle* event_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  ClockEvent* event = static_cast<ClockEvent*>(event_handle->ptr());
  return Napi::Boolean::New(info.Env(), event->is_set());
}

Napi::Value ClockEventSet(const Napi::CallbackInfo& info) {
  RclHandle* event_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  ClockEvent* event = static_cast<ClockEvent*>(event_handle->ptr());
  event->set();
  return info.Env().Undefined();
}

Napi::Value ClockEventClear(const Napi::CallbackInfo& info) {
  RclHandle* event_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  ClockEvent* event = static_cast<ClockEvent*>(event_handle->ptr());
  event->clear();
  return info.Env().Undefined();
}

void InitClockEventBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createClockEvent", Napi::Function::New(env, CreateClockEvent));
  exports.Set("clockEventWaitUntilSteady",
              Napi::Function::New(env, ClockEventWaitUntilSteady));
  exports.Set("clockEventWaitUntilSystem",
              Napi::Function::New(env, ClockEventWaitUntilSystem));
  exports.Set("clockEventWaitUntilRos",
              Napi::Function::New(env, ClockEventWaitUntilRos));
  exports.Set("clockEventIsSet", Napi::Function::New(env, ClockEventIsSet));
  exports.Set("clockEventSet", Napi::Function::New(env, ClockEventSet));
  exports.Set("clockEventClear", Napi::Function::New(env, ClockEventClear));
}

}  // namespace rclnodejs
