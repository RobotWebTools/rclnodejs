#ifndef RCLNODEJS_TYPE_CONVERSION_HPP_
#define RCLNODEJS_TYPE_CONVERSION_HPP_

#include <nan.h>
#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
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
  return val->IntegerValue(Nan::GetCurrentContext()).ToChecked();
}

template <>
inline uint64_t ToNativeChecked<uint64_t>(v8::Local<v8::Value> val) {
  return static_cast<uint64_t>(
      val->IntegerValue(Nan::GetCurrentContext()).ToChecked());
}

template <>
inline float ToNativeChecked<float>(v8::Local<v8::Value> val) {
  return Nan::To<double>(std::move(val)).ToChecked();
}

template <>
inline rosidl_runtime_c__String ToNativeChecked<rosidl_runtime_c__String>(
    v8::Local<v8::Value> val) {
  rosidl_runtime_c__String ros_string;
  rosidl_runtime_c__String__init(&ros_string);
  if (!val->IsString()) {
    return ros_string;
  };
  Nan::Utf8String utf8(val);
  if (*utf8 == nullptr) {
    throw std::runtime_error("failed to convert value to string");
  }
  rosidl_runtime_c__String__assign(&ros_string, *utf8);
  return ros_string;
}

template <>
inline rosidl_runtime_c__U16String ToNativeChecked<rosidl_runtime_c__U16String>(
    v8::Local<v8::Value> val) {
  rosidl_runtime_c__U16String ros_string;
  rosidl_runtime_c__U16String__init(&ros_string);
  if (!val->IsString()) {
    return ros_string;
  };
  Nan::Utf8String utf8(val);
  if (*utf8 == nullptr) {
    throw std::runtime_error("failed to convert value to string");
  }
  auto u16s = string_to_u16string(*utf8);
  rosidl_runtime_c__U16String__assign(
      &ros_string, reinterpret_cast<const uint16_t*>(u16s.c_str()));
  return ros_string;
}

template <typename TypedArrayT, typename NativeT>
inline void _WriteNativeArrayImpl(v8::Local<v8::Value> val, NativeT* arr,
                                  size_t len) {
  if (!val->IsTypedArray()) {
    if (!val->IsArray()) {
      return;
    }
    auto array = val.As<v8::Array>();
    if (array->Length() < len) {
      len = array->Length();
    }
    for (uint32_t i = 0; i < len; ++i) {
      auto native =
          ToNativeChecked<NativeT>(Nan::Get(array, i).ToLocalChecked());
      arr[i] = native;
    }
  } else {
    auto typed_array = val.As<v8::TypedArray>();
    size_t copy_len = typed_array->ByteLength() < len * sizeof(NativeT)
                          ? typed_array->ByteLength()
                          : len * sizeof(NativeT);
    typed_array->CopyContents(arr, copy_len);
  }
}

template <typename T>
inline void WriteNativeArray(v8::Local<v8::Value> val, T* arr, size_t len) {
  if (!val->IsArray()) {
    return;
  }
  auto array = val.As<v8::Array>();
  if (array->Length() < len) {
    len = array->Length();
  }
  for (size_t i = 0; i < len; ++i) {
    auto item = Nan::Get(array, i).ToLocalChecked();
    arr[i] = ToNativeChecked<T>(std::move(item));
  }
}

template <>
inline void WriteNativeArray<int8_t>(v8::Local<v8::Value> val, int8_t* arr,
                                     size_t len) {
  _WriteNativeArrayImpl<v8::Int8Array, int8_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint8_t>(v8::Local<v8::Value> val, uint8_t* arr,
                                      size_t len) {
  _WriteNativeArrayImpl<v8::Uint8Array, uint8_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<int16_t>(v8::Local<v8::Value> val, int16_t* arr,
                                      size_t len) {
  _WriteNativeArrayImpl<v8::Int16Array, int16_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint16_t>(v8::Local<v8::Value> val, uint16_t* arr,
                                       size_t len) {
  _WriteNativeArrayImpl<v8::Uint16Array, uint16_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<int32_t>(v8::Local<v8::Value> val, int32_t* arr,
                                      size_t len) {
  _WriteNativeArrayImpl<v8::Int32Array, int32_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint32_t>(v8::Local<v8::Value> val, uint32_t* arr,
                                       size_t len) {
  _WriteNativeArrayImpl<v8::Uint32Array, uint32_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<int64_t>(v8::Local<v8::Value> val, int64_t* arr,
                                      size_t len) {
  _WriteNativeArrayImpl<v8::BigInt64Array, int64_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<uint64_t>(v8::Local<v8::Value> val, uint64_t* arr,
                                       size_t len) {
  _WriteNativeArrayImpl<v8::BigUint64Array, uint64_t>(val, arr, len);
}

template <>
inline void WriteNativeArray<float>(v8::Local<v8::Value> val, float* arr,
                                    size_t len) {
  _WriteNativeArrayImpl<v8::Float32Array, float>(val, arr, len);
}

template <>
inline void WriteNativeArray<double>(v8::Local<v8::Value> val, double* arr,
                                     size_t len) {
  _WriteNativeArrayImpl<v8::Float64Array, double>(val, arr, len);
}

template <typename T>
inline void WriteNativeObjectArray(
    const v8::Local<v8::Value>& val, T* arr, size_t len,
    const v8::Local<v8::Object>& typesupport_msg) {
  if (!val->IsArray()) {
    return;
  }
  auto array = val.As<v8::Array>();
  if (array->Length() < len) {
    len = array->Length();
  }
  v8::Local<v8::Value> argv[2];
  auto typesupport_func =
      Nan::To<v8::Function>(
          Nan::Get(typesupport_msg,
                   Nan::New("_writeRosMessage").ToLocalChecked())
              .ToLocalChecked())
          .ToLocalChecked();
  for (size_t i = 0; i < len; ++i) {
    auto item = Nan::Get(array, i).ToLocalChecked();
    argv[0] = item;
    argv[1] = Nan::New<v8::External>(&arr[i]);
    Nan::Call(typesupport_func, Nan::New<v8::Object>(), 2, argv)
        .ToLocalChecked();
  }
}

template <typename SequenceT>
inline void InitSequence(SequenceT* seq, size_t size);

template <>
inline void InitSequence<rosidl_runtime_c__bool__Sequence>(
    rosidl_runtime_c__bool__Sequence* seq, size_t size) {
  rosidl_runtime_c__bool__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__byte__Sequence>(
    rosidl_runtime_c__byte__Sequence* seq, size_t size) {
  rosidl_runtime_c__byte__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__char__Sequence>(
    rosidl_runtime_c__char__Sequence* seq, size_t size) {
  rosidl_runtime_c__char__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__float__Sequence>(
    rosidl_runtime_c__float__Sequence* seq, size_t size) {
  rosidl_runtime_c__float__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__double__Sequence>(
    rosidl_runtime_c__double__Sequence* seq, size_t size) {
  rosidl_runtime_c__double__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__int8__Sequence>(
    rosidl_runtime_c__int8__Sequence* seq, size_t size) {
  rosidl_runtime_c__int8__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__uint8__Sequence>(
    rosidl_runtime_c__uint8__Sequence* seq, size_t size) {
  rosidl_runtime_c__uint8__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__int16__Sequence>(
    rosidl_runtime_c__int16__Sequence* seq, size_t size) {
  rosidl_runtime_c__int16__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__uint16__Sequence>(
    rosidl_runtime_c__uint16__Sequence* seq, size_t size) {
  rosidl_runtime_c__uint16__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__int32__Sequence>(
    rosidl_runtime_c__int32__Sequence* seq, size_t size) {
  rosidl_runtime_c__int32__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__uint32__Sequence>(
    rosidl_runtime_c__uint32__Sequence* seq, size_t size) {
  rosidl_runtime_c__uint32__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__int64__Sequence>(
    rosidl_runtime_c__int64__Sequence* seq, size_t size) {
  rosidl_runtime_c__int64__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__uint64__Sequence>(
    rosidl_runtime_c__uint64__Sequence* seq, size_t size) {
  rosidl_runtime_c__uint64__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__String__Sequence>(
    rosidl_runtime_c__String__Sequence* seq, size_t size) {
  rosidl_runtime_c__String__Sequence__init(seq, size);
}

template <>
inline void InitSequence<rosidl_runtime_c__U16String__Sequence>(
    rosidl_runtime_c__U16String__Sequence* seq, size_t size) {
  rosidl_runtime_c__U16String__Sequence__init(seq, size);
}

template <typename SequenceT>
inline void WriteNativeSequence(v8::Local<v8::Value> val, SequenceT* seq,
                                size_t capacity) {
  if (!val->IsTypedArray() && !val->IsArray()) {
    return;
  }
  uint32_t len;
  if (val->IsTypedArray()) {
    len = val.As<v8::TypedArray>()->Length();
  } else {
    len = val.As<v8::Array>()->Length();
  }
  if (capacity < len) {
    len = capacity;
  }
  InitSequence(seq, len);
  WriteNativeArray(val, seq->data, len);
}

template <typename SequenceT>
inline void WriteNativeObjectSequence(
    const v8::Local<v8::Value>& val, SequenceT* seq, size_t capacity,
    const v8::Local<v8::Object>& typesupport_msg) {
  if (!val->IsArray()) {
    return;
  }
  auto array = val.As<v8::Array>();

  auto len = array->Length();
  if (capacity < len) {
    len = capacity;
  }

  auto typesupport_func =
      Nan::To<v8::Function>(
          Nan::Get(typesupport_msg, Nan::New("_initSequence").ToLocalChecked())
              .ToLocalChecked())
          .ToLocalChecked();
  v8::Local<v8::Value> argv[] = {Nan::New(len), Nan::New<v8::External>(seq)};
  Nan::Call(typesupport_func, Nan::New<v8::Object>(), 2, argv).ToLocalChecked();

  WriteNativeObjectArray(val, seq->data, len, typesupport_msg);
}

template <typename T>
inline v8::Local<v8::Value> ToJsChecked(T val) {
  return Nan::New(val);
}

template <>
inline v8::Local<v8::Value> ToJsChecked<int64_t>(int64_t val) {
  // max/min safe integer
  if (val > 9007199254740991 || val < -9007199254740991) {
    return v8::BigInt::New(Nan::GetCurrentContext()->GetIsolate(), val);
  }
  return Nan::New<v8::Number>(val);
}

template <>
inline v8::Local<v8::Value> ToJsChecked<uint64_t>(uint64_t val) {
  // max/min safe integer
  if (val > 9007199254740991) {
    return v8::BigInt::NewFromUnsigned(Nan::GetCurrentContext()->GetIsolate(),
                                       val);
  }
  return Nan::New<v8::Number>(val);
}

template <>
inline v8::Local<v8::Value> ToJsChecked<rosidl_runtime_c__String>(
    rosidl_runtime_c__String val) {
  return Nan::New(val.data).ToLocalChecked();
}

template <>
inline v8::Local<v8::Value> ToJsChecked<rosidl_runtime_c__U16String>(
    rosidl_runtime_c__U16String val) {
  return Nan::New(u16string_to_string(reinterpret_cast<char16_t*>(val.data)))
      .ToLocalChecked();
}

inline uint32_t JsArrayMaxLength() {
  return std::numeric_limits<uint32_t>::max();
}

template <typename T>
inline v8::Local<v8::Value> ToJsArrayChecked(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  auto js_array = Nan::New<v8::Array>();
  auto* data = reinterpret_cast<char*>(buffer->GetContents().Data());
  auto* arr = reinterpret_cast<T*>(data + offset);
  uint32_t js_len = len > JsArrayMaxLength() ? JsArrayMaxLength() : len;
  for (uint32_t i = 0; i < js_len; i++) {
    Nan::Set(js_array, i, ToJsChecked(arr[i]));
  }
  return js_array;
}

template <typename TypedArrayT>
inline v8::Local<v8::TypedArray> _ToJsTypedArray(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return TypedArrayT::New(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<int8_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Int8Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<uint8_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Uint8Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<int16_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Int16Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<uint16_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Uint16Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<int32_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Int32Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<uint32_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Uint32Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<int64_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::BigInt64Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<uint64_t>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::BigUint64Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<float>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Float32Array>(buffer, offset, len);
}

template <>
inline v8::Local<v8::Value> ToJsArrayChecked<double>(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len) {
  return _ToJsTypedArray<v8::Float64Array>(buffer, offset, len);
}

template <typename T>
inline v8::Local<v8::Value> ToJsObjectArrayChecked(
    const v8::Local<v8::ArrayBuffer>& buffer, size_t offset, size_t len,
    const v8::Local<v8::Object>& typesupport_msg) {
  auto js_array = Nan::New<v8::Array>();
  uint32_t js_len = len > JsArrayMaxLength() ? JsArrayMaxLength() : len;
  auto typesupport_func =
      Nan::To<v8::Function>(
          Nan::Get(typesupport_msg, Nan::New("_toJsObject").ToLocalChecked())
              .ToLocalChecked())
          .ToLocalChecked();
  v8::Local<v8::Value> argv[1];
  for (uint32_t i = 0; i < js_len; ++i) {
    argv[0] = v8::Uint8Array::New(buffer, offset, len * sizeof(T));
    auto js_obj = Nan::Call(typesupport_func, Nan::New<v8::Object>(), 1, argv)
                      .ToLocalChecked();
    Nan::Set(js_array, i, js_obj);
  }
  return js_array;
}

}  // namespace rclnodejs

#endif
