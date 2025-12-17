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

#ifndef SRC_CLOCK_EVENT_HPP_
#define SRC_CLOCK_EVENT_HPP_

#include <napi.h>
#include <rcl/time.h>

#include <condition_variable>
#include <memory>
#include <mutex>

namespace rclnodejs {

class ClockEvent {
 public:
  /// Wait until a time specified by a system or steady clock.
  /// \param clock the clock to use for time synchronization with until
  /// \param until this method will block until this time is reached.
  template <typename ClockType>
  void wait_until(rcl_clock_t* clock, rcl_time_point_t until);

  /// Wait until a time specified by a ROS clock.
  /// \warning The caller is responsible for creating a time jump callback
  /// to set this event when the target ROS time is reached.
  /// \param clock The clock to use for time synchronization.
  /// \param until This method will block until this time is reached.
  void wait_until_ros(rcl_clock_t* clock, rcl_time_point_t until);

  /// Indicate if the ClockEvent is set.
  /// \return True if the ClockEvent is set.
  bool is_set();

  /// Set the event.
  void set();

  /// Clear the event.
  void clear();

 private:
  bool state_ = false;
  std::mutex mutex_;
  std::condition_variable cv_;
};

void InitClockEventBindings(Napi::Env env, Napi::Object exports);

}  // namespace rclnodejs

#endif  // SRC_CLOCK_EVENT_HPP_
