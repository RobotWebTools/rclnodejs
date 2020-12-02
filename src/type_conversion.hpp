#ifndef RCLNODEJS_TYPE_CONVERSION_HPP_
#define RCLNODEJS_TYPE_CONVERSION_HPP_

#include <nan.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/u16string.h>
#include <rosidl_runtime_c/u16string_functions.h>

namespace rclnodejs {

template <typename T>
inline T ToNativeChecked(v8::Local<v8::Value> val) {
  return Nan::To<T>(std::move(val)).ToChecked();
}

template <>
inline int8_t ToNativeChecked<int8_t>(v8::Local<v8::Value> val) {
  return Nan::To<int32_t>(std::move(val)).ToChecked();
}

template <>
inline uint8_t ToNativeChecked<uint8_t>(v8::Local<v8::Value> val) {
  return Nan::To<int32_t>(std::move(val)).ToChecked();
}

template <>
inline int16_t ToNativeChecked<int16_t>(v8::Local<v8::Value> val) {
  return Nan::To<int32_t>(std::move(val)).ToChecked();
}

template <>
inline uint16_t ToNativeChecked<uint16_t>(v8::Local<v8::Value> val) {
  return Nan::To<int32_t>(std::move(val)).ToChecked();
}

template <>
inline int64_t ToNativeChecked<int64_t>(v8::Local<v8::Value> val) {
  return val->ToBigInt(Nan::GetCurrentContext()).ToLocalChecked()->Int64Value();
}

template <>
inline uint64_t ToNativeChecked<uint64_t>(v8::Local<v8::Value> val) {
  return val->ToBigInt(Nan::GetCurrentContext())
      .ToLocalChecked()
      ->Uint64Value();
}

template <>
inline float ToNativeChecked<float>(v8::Local<v8::Value> val) {
  return Nan::To<double>(std::move(val)).ToChecked();
}

template <>
inline rosidl_runtime_c__String ToNativeChecked<rosidl_runtime_c__String>(
    v8::Local<v8::Value> val) {
  Nan::Utf8String utf8(val);
  if (*utf8 == nullptr) {
    throw std::runtime_error("failed to convert value to string");
  }
  rosidl_runtime_c__String ros_string;
  rosidl_runtime_c__String__assign(&ros_string, *utf8);
  return ros_string;
}

template <>
inline rosidl_runtime_c__U16String ToNativeChecked<rosidl_runtime_c__U16String>(
    v8::Local<v8::Value> val) {
  // TODO:
  throw std::runtime_error("not implemented");
}

template <typename T>
inline v8::Local<v8::Value> ToJsChecked(T val) {
  return Nan::New(val);
}

template <>
inline v8::Local<v8::Value> ToJsChecked<int64_t>(int64_t val) {
  return v8::BigInt::New(Nan::GetCurrentContext()->GetIsolate(), val);
}

template <>
inline v8::Local<v8::Value> ToJsChecked<uint64_t>(uint64_t val) {
  return v8::BigInt::NewFromUnsigned(Nan::GetCurrentContext()->GetIsolate(),
                                     val);
}

// can't specialize const ref, probably need to use SFINAE
template <>
inline v8::Local<v8::Value> ToJsChecked<rosidl_runtime_c__String>(
    rosidl_runtime_c__String val) {
  return Nan::New(val.data).ToLocalChecked();
}

// can't specialize const ref, probably need to use SFINAE
template <>
inline v8::Local<v8::Value> ToJsChecked<rosidl_runtime_c__U16String>(
    rosidl_runtime_c__U16String val) {
  // TODO:
  throw new std::runtime_error("not implemented");
}

}  // namespace rclnodejs

#endif
