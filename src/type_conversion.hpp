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

class OutOfRangeError : public std::exception {
 public:
  explicit OutOfRangeError(size_t len) : _len(len) {
    _what = "expected array to have length " + len;
  }

  const char* what() const noexcept override { return _what.c_str(); }

  std::string what_detailed(const std::string& field_name) {
    return "expected \"" + field_name + "\" to have length " +
           std::to_string(_len);
  }

 private:
  std::string _what;
  size_t _len;
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
inline T ToNativeChecked(v8::Local<v8::Value> val) {
  return Nan::To<T>(std::move(val)).ToChecked();
}

template <>
inline int8_t ToNativeChecked<int8_t>(v8::Local<v8::Value> val) {
  return Nan::To<int32_t>(std::move(val)).ToChecked();
}

template <>
inline uint8_t ToNativeChecked<uint8_t>(v8::Local<v8::Value> val) {
  return Nan::To<uint32_t>(std::move(val)).ToChecked();
}

template <>
inline int16_t ToNativeChecked<int16_t>(v8::Local<v8::Value> val) {
  return Nan::To<int32_t>(std::move(val)).ToChecked();
}

template <>
inline uint16_t ToNativeChecked<uint16_t>(v8::Local<v8::Value> val) {
  return Nan::To<uint32_t>(std::move(val)).ToChecked();
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
  rosidl_runtime_c__String__init(&ros_string);
  rosidl_runtime_c__String__assign(&ros_string, *utf8);
  return ros_string;
}

template <>
inline rosidl_runtime_c__U16String ToNativeChecked<rosidl_runtime_c__U16String>(
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

template <typename TypedArrayT>
struct TypedArrayName {};

template <>
struct TypedArrayName<v8::Int8Array> {
  static constexpr const char* Name = "Int8Array";
};

template <>
struct TypedArrayName<v8::Uint8Array> {
  static constexpr const char* Name = "Uint8Array";
};

template <>
struct TypedArrayName<v8::Int16Array> {
  static constexpr const char* Name = "Int16Array";
};

template <>
struct TypedArrayName<v8::Uint16Array> {
  static constexpr const char* Name = "Uint16Array";
};

template <>
struct TypedArrayName<v8::Int32Array> {
  static constexpr const char* Name = "Int32Array";
};

template <>
struct TypedArrayName<v8::Uint32Array> {
  static constexpr const char* Name = "Uint32Array";
};

template <>
struct TypedArrayName<v8::BigInt64Array> {
  static constexpr const char* Name = "BigInt64Array";
};

template <>
struct TypedArrayName<v8::BigUint64Array> {
  static constexpr const char* Name = "BigUint64Array";
};

template <typename TypedArrayT>
inline bool IsTypedArray(v8::Local<v8::Value> val);

template <>
inline bool IsTypedArray<v8::Int8Array>(v8::Local<v8::Value> val) {
  return val->IsInt8Array();
}

template <>
inline bool IsTypedArray<v8::Uint8Array>(v8::Local<v8::Value> val) {
  return val->IsUint8Array();
}

template <>
inline bool IsTypedArray<v8::Int16Array>(v8::Local<v8::Value> val) {
  return val->IsInt16Array();
}

template <>
inline bool IsTypedArray<v8::Uint16Array>(v8::Local<v8::Value> val) {
  return val->IsUint16Array();
}

template <>
inline bool IsTypedArray<v8::Int32Array>(v8::Local<v8::Value> val) {
  return val->IsInt32Array();
}

template <>
inline bool IsTypedArray<v8::Uint32Array>(v8::Local<v8::Value> val) {
  return val->IsUint32Array();
}

template <>
inline bool IsTypedArray<v8::BigInt64Array>(v8::Local<v8::Value> val) {
  return val->IsBigInt64Array();
}

template <>
inline bool IsTypedArray<v8::BigUint64Array>(v8::Local<v8::Value> val) {
  return val->IsBigUint64Array();
}

template <typename TypedArrayT, typename NativeT>
inline void WriteNativeArrayImpl(v8::Local<v8::Value> val, NativeT* arr,
                                 size_t len) {
  if (!IsTypedArray<TypedArrayT>(val)) {
    if (!val->IsArray()) {
      throw TypeError({"array", TypedArrayName<TypedArrayT>::Name});
    }
    auto array = val.As<v8::Array>();
    if (array->Length() < len) {
      throw OutOfRangeError(len);
    }
    for (uint32_t i = 0; i < len; ++i) {
      auto native =
          ToNativeChecked<NativeT>(Nan::Get(array, i).ToLocalChecked());
      arr[i] = native;
    }
  } else {
    auto typed_array = val.As<TypedArrayT>();
    if (typed_array->Length() < len) {
      throw OutOfRangeError(len);
    }
    typed_array->CopyContents(arr, len * sizeof(NativeT));
  }
}

template <typename T>
inline void WriteNativeArray(v8::Local<v8::Value> val, T* arr, size_t len);

template <>
inline void WriteNativeArray<int8_t>(v8::Local<v8::Value> val, int8_t* arr,
                                     size_t len) {
  WriteNativeArrayImpl<v8::Int8Array, int8_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint8_t>(v8::Local<v8::Value> val, uint8_t* arr,
                                      size_t len) {
  WriteNativeArrayImpl<v8::Uint8Array, uint8_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<int16_t>(v8::Local<v8::Value> val, int16_t* arr,
                                      size_t len) {
  WriteNativeArrayImpl<v8::Int16Array, int16_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint16_t>(v8::Local<v8::Value> val, uint16_t* arr,
                                       size_t len) {
  WriteNativeArrayImpl<v8::Uint16Array, uint16_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<int32_t>(v8::Local<v8::Value> val, int32_t* arr,
                                      size_t len) {
  WriteNativeArrayImpl<v8::Int32Array, int32_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint32_t>(v8::Local<v8::Value> val, uint32_t* arr,
                                       size_t len) {
  WriteNativeArrayImpl<v8::Uint32Array, uint32_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<int64_t>(v8::Local<v8::Value> val, int64_t* arr,
                                      size_t len) {
  WriteNativeArrayImpl<v8::BigInt64Array, int64_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint64_t>(v8::Local<v8::Value> val, uint64_t* arr,
                                       size_t len) {
  WriteNativeArrayImpl<v8::BigUint64Array, uint64_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<bool>(v8::Local<v8::Value> val, bool* arr,
                                   size_t len) {
  if (!val->IsArray()) {
    throw TypeError({"array"});
  }
  auto array = val.As<v8::Array>();
  if (array->Length() < len) {
    throw OutOfRangeError(len);
  }
  for (size_t i = 0; i < len; ++i) {
    auto item = Nan::Get(array, i).ToLocalChecked();
    arr[i] = Nan::To<bool>(std::move(item)).ToChecked();
  }
}

template <>
inline void WriteNativeArray<float>(v8::Local<v8::Value> val, float* arr,
                                    size_t len) {
  if (!val->IsArray()) {
    throw TypeError({"array"});
  }
  auto array = val.As<v8::Array>();
  if (array->Length() < len) {
    throw OutOfRangeError(len);
  }
  for (size_t i = 0; i < len; ++i) {
    auto item = Nan::Get(array, i).ToLocalChecked();
    arr[i] = static_cast<float>(Nan::To<double>(std::move(item)).ToChecked());
  }
}

template <>
inline void WriteNativeArray<double>(v8::Local<v8::Value> val, double* arr,
                                     size_t len) {
  if (!val->IsArray()) {
    throw TypeError({"array"});
  }
  auto array = val.As<v8::Array>();
  if (array->Length() < len) {
    throw OutOfRangeError(len);
  }
  for (size_t i = 0; i < len; ++i) {
    auto item = Nan::Get(array, i).ToLocalChecked();
    arr[i] = Nan::To<double>(std::move(item)).ToChecked();
  }
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
