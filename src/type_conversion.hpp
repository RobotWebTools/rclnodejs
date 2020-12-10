#ifndef RCLNODEJS_TYPE_CONVERSION_HPP_
#define RCLNODEJS_TYPE_CONVERSION_HPP_

#include <nan.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/u16string.h>
#include <rosidl_runtime_c/u16string_functions.h>

#include <codecvt>
#include <exception>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace rclnodejs {

class TypeError : public std::exception {
 public:
  explicit TypeError(const std::vector<std::string>& expected_types) {
    std::ostringstream oss;
    for (size_t i = 0; i < expected_types.size() - 1; ++i) {
      oss << expected_types[i] << " or ";
    }
    oss << expected_types[expected_types.size() - 1];
    _expected_types = oss.str();
    _what = "expected " + _expected_types;
  }

  const char* what() const noexcept override { return _what.c_str(); }

  std::string what_detailed(const std::string& field_name) {
    return "expected \"" + field_name + "\" to be " + _expected_types;
  }

 private:
  std::string _expected_types;
  std::string _what;
};

std::u16string string_to_u16string(const std::string& input) {
  std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> converter;
  return converter.from_bytes(input);
}

std::string u16string_to_string(const std::u16string& input) {
  std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> converter;
  return converter.to_bytes(input);
}

template <typename T>
inline T ToNative(v8::Local<v8::Value> val);

template <>
inline int8_t ToNative<int8_t>(v8::Local<v8::Value> val) {
  int32_t native;
  if (!Nan::To<int32_t>(std::move(val)).To(&native) ||
      native > std::numeric_limits<int8_t>::max() ||
      native < std::numeric_limits<int8_t>::min()) {
    throw TypeError({"int8"});
  }
  return static_cast<int8_t>(native);
}

template <>
inline uint8_t ToNative<uint8_t>(v8::Local<v8::Value> val) {
  uint32_t native;
  if (!Nan::To<uint32_t>(std::move(val)).To(&native) ||
      native > std::numeric_limits<uint8_t>::max() ||
      native < std::numeric_limits<uint8_t>::min()) {
    throw TypeError({"uint8"});
  }
  return static_cast<uint8_t>(native);
}

template <>
inline int16_t ToNative<int16_t>(v8::Local<v8::Value> val) {
  int32_t native;
  if (!Nan::To<int32_t>(std::move(val)).To(&native) ||
      native > std::numeric_limits<int16_t>::max() ||
      native < std::numeric_limits<int16_t>::min()) {
    throw TypeError({"int16"});
  }
  return static_cast<int16_t>(native);
}

template <>
inline uint16_t ToNative<uint16_t>(v8::Local<v8::Value> val) {
  uint32_t native;
  if (!Nan::To<uint32_t>(std::move(val)).To(&native) ||
      native > std::numeric_limits<uint16_t>::max() ||
      native < std::numeric_limits<uint16_t>::min()) {
    throw TypeError({"uint16"});
  }
  return static_cast<uint16_t>(native);
}

template <>
inline int32_t ToNative<int32_t>(v8::Local<v8::Value> val) {
  int32_t native;
  if (!Nan::To<int32_t>(std::move(val)).To(&native)) {
    throw TypeError({"int32"});
  }
  return native;
}

template <>
inline uint32_t ToNative<uint32_t>(v8::Local<v8::Value> val) {
  uint32_t native;
  if (!Nan::To<uint32_t>(std::move(val)).To(&native)) {
    throw TypeError({"uint32"});
  }
  return native;
}

template <>
inline int64_t ToNative<int64_t>(v8::Local<v8::Value> val) {
  v8::Local<v8::BigInt> bigint;
  if (!val->ToBigInt(Nan::GetCurrentContext()).ToLocal(&bigint)) {
    throw TypeError({"int64"});
  }
  return bigint->Int64Value();
}

template <>
inline uint64_t ToNative<uint64_t>(v8::Local<v8::Value> val) {
  v8::Local<v8::BigInt> bigint;
  if (!val->ToBigInt(Nan::GetCurrentContext()).ToLocal(&bigint)) {
    throw TypeError({"unt64"});
  }
  return bigint->Uint64Value();
}

template <>
inline float ToNative<float>(v8::Local<v8::Value> val) {
  double native;
  if (!Nan::To<double>(std::move(val)).To(&native) ||
      native > std::numeric_limits<float>::max() ||
      native < std::numeric_limits<float>::min()) {
    throw TypeError({"float"});
  }
  return static_cast<float>(native);
}

template <>
inline double ToNative<double>(v8::Local<v8::Value> val) {
  double native;
  if (!Nan::To<double>(std::move(val)).To(&native)) {
    throw TypeError({"double"});
  }
  return native;
}

template <>
inline bool ToNative<bool>(v8::Local<v8::Value> val) {
  bool native;
  if (!Nan::To<bool>(std::move(val)).To(&native)) {
    throw TypeError({"bool"});
  }
  return native;
}

template <>
inline rosidl_runtime_c__String ToNative<rosidl_runtime_c__String>(
    v8::Local<v8::Value> val) {
  if (!val->IsString()) {
    throw TypeError({"string"});
  };
  Nan::Utf8String utf8(val);
  if (*utf8 == nullptr) {
    throw std::runtime_error("failed to convert value to string");
  }
  rosidl_runtime_c__String ros_string;
  rosidl_runtime_c__String__init(&ros_string);
  rosidl_runtime_c__String__assign(&ros_string, *utf8);
  return ros_string;
}

template <>
inline rosidl_runtime_c__U16String ToNative<rosidl_runtime_c__U16String>(
    v8::Local<v8::Value> val) {
  if (!val->IsString()) {
    throw TypeError({"string"});
  };
  Nan::Utf8String utf8(val);
  if (*utf8 == nullptr) {
    throw std::runtime_error("failed to convert value to string");
  }
  auto u16s = string_to_u16string(*utf8);
  rosidl_runtime_c__U16String ros_string;
  rosidl_runtime_c__U16String__init(&ros_string);
  rosidl_runtime_c__U16String__assign(
      &ros_string, reinterpret_cast<const uint16_t*>(u16s.c_str()));
  return ros_string;
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
