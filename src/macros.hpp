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

#ifndef RCLNODEJS_MARCOS_HPP_
#define RCLNODEJS_MARCOS_HPP_

#include "rcutils/logging_macros.h"

#define CHECK_OP_AND_THROW_ERROR_IF_NOT_TRUE(op, lhs, rhs, message) \
  {                                                                 \
    if (lhs op rhs) {                                               \
      Nan::ThrowError(message);                                     \
      rcl_reset_error();                                            \
      return;                                                       \
    }                                                               \
  }

#define THROW_ERROR_IF_NOT_EQUAL(lhs, rhs, message) \
  CHECK_OP_AND_THROW_ERROR_IF_NOT_TRUE(!=, lhs, rhs, message)

#define THROW_ERROR_IF_EQUAL(lhs, rhs, message) \
  CHECK_OP_AND_THROW_ERROR_IF_NOT_TRUE(==, lhs, rhs, message)

#define PACKAGE_NAME "rclnodejs"

#ifdef DEBUG_ON
#define RCLNODEJS_DEBUG(...)                                   \
  RCUTILS_LOG_COND_NAMED(                                      \
      RCUTILS_LOG_SEVERITY_DEBUG, RCUTILS_LOG_CONDITION_EMPTY, \
      RCUTILS_LOG_CONDITION_EMPTY, PACKAGE_NAME, __VA_ARGS__)
#else
#define RCLNODEJS_DEBUG(...)
#endif

#endif
