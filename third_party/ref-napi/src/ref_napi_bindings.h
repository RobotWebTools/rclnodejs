// Copyright (c) 2020 The ref-napi Authors.
//
// Licensed under the MIT License. See LICENSE in third_party/ref-napi.
//
// This file adapts the ref-napi addon initialization for integration
// into the rclnodejs native module.

#ifndef THIRD_PARTY_REF_NAPI_SRC_REF_NAPI_BINDINGS_H_
#define THIRD_PARTY_REF_NAPI_SRC_REF_NAPI_BINDINGS_H_

#include <napi.h>

namespace rclnodejs {

Napi::Object InitRefNapi(Napi::Env env);

}  // namespace rclnodejs

#endif  // THIRD_PARTY_REF_NAPI_SRC_REF_NAPI_BINDINGS_H_
