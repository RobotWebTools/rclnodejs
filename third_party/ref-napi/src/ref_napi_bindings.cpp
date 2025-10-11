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

#include "ref_napi_bindings.h"

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <unordered_map>

#if !defined(NAPI_VERSION) || NAPI_VERSION < 6
#include <get-symbol-from-current-process.h>
#endif

namespace RefNapi {

class Instance {
 public:
  virtual napi_value WrapPointer(char* ptr, size_t length) = 0;
  virtual char* GetBufferData(napi_value val) = 0;
};

}  // namespace RefNapi

#ifndef _WIN32
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>
#else
#define __alignof__ __alignof
#define snprintf(buf, bufSize, format, arg) \
  _snprintf_s(buf, bufSize, _TRUNCATE, format, arg)
#define strtoll _strtoi64
#define strtoull _strtoui64
#define PRId64 "lld"
#define PRIu64 "llu"
#endif

namespace rclnodejs {

using namespace Napi;

namespace {

// used by the Int64 functions to determine whether to return a Number
// or String based on whether or not a Number will lose precision.
// http://stackoverflow.com/q/307179/376773
#define JS_MAX_INT +9007199254740992LL
#define JS_MIN_INT -9007199254740992LL

// mirrors deps/v8/src/objects.h.
// we could use `node::Buffer::kMaxLength`, but it's not defined on node v0.6.x
static const size_t kMaxLength = 0x3fffffff;

// Since Node.js v14.0.0, we have to keep a global list of all ArrayBuffer
// instances that we work with, in order not to create any duplicates.
// Luckily, N-API instance data is available on v14.x and above.
class InstanceData final : public RefNapi::Instance {
 public:
  explicit InstanceData(Env env) : env(env) {}
  Env env;
  FunctionReference pointer_ctor;

  napi_value WrapPointer(char* ptr, size_t length) override;
  char* GetBufferData(napi_value val) override;

  static InstanceData* Get(Env env) {
    return env.GetInstanceData<InstanceData>();
  }
};

class PointerBuffer : public ObjectWrap<PointerBuffer> {
 public:
  static Object Init(Napi::Env env, Object exports);
  explicit PointerBuffer(Napi::CallbackInfo& info);
  Napi::Value IsNull(const Napi::CallbackInfo& info);
  Napi::Value Address(const Napi::CallbackInfo& info);
  Napi::Value Length(const Napi::CallbackInfo& info);
  Napi::Value Get(const Napi::CallbackInfo& info);
  Napi::Value ToString(const Napi::CallbackInfo& info);
  Napi::Value Copy(const Napi::CallbackInfo& info);
  Napi::Value Slice(const Napi::CallbackInfo& info);
  char* ptr_;
  int length_;
};

Object PointerBuffer::Init(Napi::Env env, Object exports) {
  Function func =
      DefineClass(env, "PointerBuffer",
                  {InstanceMethod("isNull", &PointerBuffer::IsNull),
                   InstanceMethod("get", &PointerBuffer::Get),
                   InstanceMethod("address", &PointerBuffer::Address),
                   InstanceMethod("toString", &PointerBuffer::ToString),
                   InstanceMethod("copy", &PointerBuffer::Copy),
                   InstanceMethod("slice", &PointerBuffer::Slice),
                   InstanceAccessor<&PointerBuffer::Length>("length")});

  exports.Set("PointerBuffer", func);
  InstanceData* data = InstanceData::Get(env);
  data->pointer_ctor = Persistent(func);
  return exports;
}

PointerBuffer::PointerBuffer(Napi::CallbackInfo& info)
    : Napi::ObjectWrap<PointerBuffer>(info) {
  Napi::Env env = info.Env();
  int length = info.Length();
  if (length <= 0 || !info[0].IsNumber()) {
    Napi::TypeError::New(env, "Number expected").ThrowAsJavaScriptException();
    return;
  }
  Napi::Number value = info[0].As<Napi::Number>();
  ptr_ = reinterpret_cast<char*>(value.Int64Value());
  length_ = (ptr_ == nullptr ? 0 : info[1].As<Number>().Int32Value());
}

Napi::Value PointerBuffer::IsNull(const Napi::CallbackInfo& info) {
  return Boolean::New(info.Env(), ptr_ == nullptr);
}

Napi::Value PointerBuffer::Address(const Napi::CallbackInfo& info) {
  return Number::New(info.Env(), reinterpret_cast<int64_t>(ptr_));
}

Napi::Value PointerBuffer::Length(const Napi::CallbackInfo& info) {
  return Number::New(info.Env(), length_);
}

Napi::Value PointerBuffer::Get(const Napi::CallbackInfo& info) {
  int32_t offset = info[0].As<Number>().Int32Value();
  return Number::New(info.Env(), ptr_[offset]);
}

Napi::Value PointerBuffer::ToString(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  int length = info.Length();
  if (length == 1 || info[0].IsString()) {
    std::string encoding = info[0].As<String>();
    if (encoding == "utf-8") {
      return String::New(info.Env(), ptr_, length_);
    } else if (encoding == "ucs2") {
      return String::New(info.Env(), reinterpret_cast<char16_t*>(ptr_),
                         length_ / 2);
    } else {
      Napi::TypeError::New(env, "Unknown encoding argument: " + encoding)
          .ThrowAsJavaScriptException();
    }
  }

  return String::New(info.Env(), ptr_, length_);
}

char* ExtractBufferData(Value val);

Napi::Value PointerBuffer::Copy(const Napi::CallbackInfo& info) {
  char* dest =
      ExtractBufferData(info[0]) + info[1].As<Napi::Number>().Int64Value();
  uint64_t length =
      info[3].As<Number>().Int64Value() - info[2].As<Number>().Int64Value();
  std::memcpy(dest, ptr_, length);
  return Number::New(info.Env(), length);
}

Value CreatePointerBuffer(Env env, char* ptr, size_t length);

Napi::Value PointerBuffer::Slice(const Napi::CallbackInfo& info) {
  int64_t offset = info[0].As<Napi::Number>().Int64Value();
  return CreatePointerBuffer(info.Env(), ptr_ + offset, length_ - offset);
}

Value CreatePointerBuffer(Env env, char* ptr, size_t length) {
  InstanceData* data = InstanceData::Get(env);
  return data->pointer_ctor.New(
      {Number::New(env, reinterpret_cast<int64_t>(ptr)),
       Number::New(env, length)});
}

char* ExtractBufferData(Value val) {
  if (!val.IsBuffer() && val.IsObject()) {
    auto p = PointerBuffer::Unwrap(val.As<Object>());
    return p->ptr_;
  }

  Buffer<char> buf = val.As<Buffer<char>>();
  return buf.Data();
}

napi_value InstanceData::WrapPointer(char* ptr, size_t length) {
  return CreatePointerBuffer(env, ptr, length);
}

char* InstanceData::GetBufferData(napi_value val) {
  return ExtractBufferData(Value(env, val));
}

char* AddressForArgs(const CallbackInfo& args, size_t offset_index = 1) {
  Value buf = args[0];
  if (!(buf.IsBuffer() || buf.IsObject())) {
    throw TypeError::New(args.Env(),
                         "Buffer or PointerBuffer instance expected");
  }

  int64_t offset = args[offset_index].ToNumber();
  return ExtractBufferData(buf) + offset;
}

Value Address(const CallbackInfo& args) {
  char* ptr = AddressForArgs(args);
  uintptr_t intptr = reinterpret_cast<uintptr_t>(ptr);

  return Number::New(args.Env(), static_cast<double>(intptr));
}

Value HexAddress(const CallbackInfo& args) {
  char* ptr = AddressForArgs(args);
  char strbuf[30];
  snprintf(strbuf, 30, "%p", ptr);

  if (strbuf[0] == '0' && strbuf[1] == 'x') {
    ptr = strbuf + 2;
  } else {
    ptr = strbuf;
  }

  return String::New(args.Env(), ptr);
}

Value IsNull(const CallbackInfo& args) {
  Value buf = args[0];
  if (!(buf.IsBuffer() || buf.IsObject())) {
    return Boolean::New(args.Env(), false);
  }

  char* ptr = AddressForArgs(args);
  return Boolean::New(args.Env(), ptr == nullptr);
}

Value IsAddress(const CallbackInfo& args) {
  Value buf = args[0];
  if (!(buf.IsBuffer() || buf.IsObject())) {
    return Boolean::New(args.Env(), false);
  }
  return Boolean::New(args.Env(), true);
}

Value ReadObject(const CallbackInfo& args) {
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw Error::New(args.Env(),
                     "readObject: Cannot read from nullptr pointer");
  }

  Reference<Object>* rptr = reinterpret_cast<Reference<Object>*>(ptr);
  return rptr->Value();
}

void WriteObject(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw Error::New(env, "readObject: Cannot write to nullptr pointer");
  }

  Reference<Object>* rptr = reinterpret_cast<Reference<Object>*>(ptr);
  if (args[2].IsObject()) {
    Object val = args[2].As<Object>();
    *rptr = std::move(Reference<Object>::New(val));
  } else if (args[2].IsNull()) {
    rptr->Reset();
  } else {
    throw TypeError::New(env,
                         "WriteObject's 3rd argument needs to be an object");
  }
}

Value ReadPointer(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw Error::New(env, "readPointer: Cannot read from nullptr pointer");
  }

  int64_t size = args[2].ToNumber();

  char* val = *reinterpret_cast<char**>(ptr);
  return CreatePointerBuffer(env, val, size);
}

void WritePointer(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);
  Value input = args[2];

  if (!input.IsNull() && !input.IsBuffer() && !input.IsObject()) {
    throw TypeError::New(
        env, "writePointer: Buffer instance expected as third argument");
  }

  if (input.IsNull()) {
    *reinterpret_cast<char**>(ptr) = nullptr;
  } else {
    char* input_ptr = ExtractBufferData(input);
    *reinterpret_cast<char**>(ptr) = input_ptr;
  }
}

Value ReadInt64(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readInt64: Cannot read from nullptr pointer");
  }

  int64_t val = *reinterpret_cast<int64_t*>(ptr);

  if (val < JS_MIN_INT || val > JS_MAX_INT) {
    char strbuf[128];
    snprintf(strbuf, 128, "%" PRId64, val);
    return String::New(env, strbuf);
  } else {
    return Number::New(env, val);
  }
}

Value ReadInt32(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readInt64: Cannot read from nullptr pointer");
  }

  int32_t val = *reinterpret_cast<int32_t*>(ptr);
  return Number::New(env, val);
}

void WriteInt64(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  Value in = args[2];
  int64_t val;
  if (in.IsNumber()) {
    val = in.As<Number>();
  } else if (in.IsString()) {
    char* endptr;
    char* str;
    int base = 0;
    std::string _str = in.As<String>();
    str = &_str[0];

    errno = 0;
    val = strtoll(str, &endptr, base);

    if (endptr == str) {
      throw TypeError::New(env,
                           "writeInt64: no digits we found in input String");
    } else if (errno == ERANGE && (val == INT64_MAX || val == INT64_MIN)) {
      throw TypeError::New(
          env, "writeInt64: input String numerical value out of range");
    } else if (errno != 0 && val == 0) {
      char errmsg[200];
      snprintf(errmsg, sizeof(errmsg), "writeInt64: %s", strerror(errno));
      throw TypeError::New(env, errmsg);
    }
  } else {
    throw TypeError::New(env,
                         "writeInt64: Number/String 64-bit value required");
  }

  *reinterpret_cast<int64_t*>(ptr) = val;
}

void WriteInt32(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  Value in = args[2];
  int64_t val;
  if (in.IsNumber()) {
    val = in.As<Number>();
  } else if (in.IsString()) {
    char* endptr;
    char* str;
    int base = 0;
    std::string _str = in.As<String>();
    str = &_str[0];

    errno = 0;
    val = strtoll(str, &endptr, base);

    if (endptr == str) {
      throw TypeError::New(env,
                           "writeInt32: no digits we found in input String");
    } else if (errno == ERANGE && (val == INT32_MAX || val == INT32_MIN)) {
      throw TypeError::New(
          env, "writeInt32: input String numerical value out of range");
    } else if (errno != 0 && val == 0) {
      char errmsg[200];
      snprintf(errmsg, sizeof(errmsg), "writeInt32: %s", strerror(errno));
      throw TypeError::New(env, errmsg);
    }
  } else {
    throw TypeError::New(env,
                         "writeInt32: Number/String 32-bit value required");
  }

  if (val < INT32_MIN || val > INT32_MAX) {
    throw TypeError::New(env, "writeInt32: value out of range");
  }

  *reinterpret_cast<int32_t*>(ptr) = static_cast<int32_t>(val);
}

Value ReadUInt32(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readUInt32: Cannot read from nullptr pointer");
  }

  uint32_t val = *reinterpret_cast<uint32_t*>(ptr);
  return Number::New(env, val);
}

Value ReadInt8(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readInt8: Cannot read from nullptr pointer");
  }

  int8_t val = *reinterpret_cast<int8_t*>(ptr);
  return Number::New(env, val);
}

Value ReadUInt8(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readUInt8: Cannot read from nullptr pointer");
  }

  uint8_t val = *reinterpret_cast<uint8_t*>(ptr);
  return Number::New(env, val);
}

Value ReadFloat(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readFloat: Cannot read from nullptr pointer");
  }

  float val = *reinterpret_cast<float*>(ptr);
  return Number::New(env, val);
}

Value ReadDouble(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readDouble: Cannot read from nullptr pointer");
  }

  double val = *reinterpret_cast<double*>(ptr);
  return Number::New(env, val);
}

Value ReadInt16(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readInt16: Cannot read from nullptr pointer");
  }

  int16_t val = *reinterpret_cast<int16_t*>(ptr);
  return Number::New(env, val);
}

Value ReadUInt16(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readUInt16: Cannot read from nullptr pointer");
  }

  uint16_t val = *reinterpret_cast<uint16_t*>(ptr);
  return Number::New(env, val);
}

Value ReadUInt64(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw TypeError::New(env, "readUInt64: Cannot read from nullptr pointer");
  }

  uint64_t val = *reinterpret_cast<uint64_t*>(ptr);

  if (val > JS_MAX_INT) {
    char strbuf[128];
    snprintf(strbuf, 128, "%" PRIu64, val);
    return String::New(env, strbuf);
  } else {
    return Number::New(env, val);
  }
}

void WriteUInt64(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  Value in = args[2];
  uint64_t val;
  if (in.IsNumber()) {
    val = static_cast<int64_t>(in.As<Number>());
  } else if (in.IsString()) {
    char* endptr;
    char* str;
    int base = 0;
    std::string _str = in.As<String>();
    str = &_str[0];

    errno = 0;
    val = strtoull(str, &endptr, base);

    if (endptr == str) {
      throw TypeError::New(env,
                           "writeUInt64: no digits we found in input String");
    } else if (errno == ERANGE && (val == UINT64_MAX)) {
      throw TypeError::New(
          env, "writeUInt64: input String numerical value out of range");
    } else if (errno != 0 && val == 0) {
      char errmsg[200];
      snprintf(errmsg, sizeof(errmsg), "writeUInt64: %s", strerror(errno));
      throw TypeError::New(env, errmsg);
    }
  } else {
    throw TypeError::New(env,
                         "writeUInt64: Number/String 64-bit value required");
  }

  *reinterpret_cast<uint64_t*>(ptr) = val;
}

Value ReadCString(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args);

  if (ptr == nullptr) {
    throw Error::New(env, "readCString: Cannot read from nullptr pointer");
  }

  return String::New(env, ptr);
}

Value ReinterpretBuffer(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args, 2);

  if (ptr == nullptr) {
    throw Error::New(env,
                     "reinterpret: Cannot reinterpret from nullptr pointer");
  }

  int64_t size = args[1].ToNumber();

  return CreatePointerBuffer(env, ptr, size);
}

Value ReinterpretBufferUntilZeros(const CallbackInfo& args) {
  Env env = args.Env();
  char* ptr = AddressForArgs(args, 2);

  if (ptr == nullptr) {
    throw Error::New(
        env, "reinterpretUntilZeros: Cannot reinterpret from nullptr pointer");
  }

  uint32_t numZeros = args[1].ToNumber();
  uint32_t i = 0;
  size_t size = 0;
  bool end = false;

  while (!end && size < kMaxLength) {
    end = true;
    for (i = 0; i < numZeros; i++) {
      if (ptr[size + i] != 0) {
        end = false;
        break;
      }
    }
    if (!end) {
      size += numZeros;
    }
  }

  return CreatePointerBuffer(env, ptr, size);
}

}  // namespace

Napi::Object InitRefNapi(Napi::Env env) {
  Object exports = Object::New(env);
  InstanceData* data = new InstanceData(env);
  env.SetInstanceData<InstanceData>(data);

  exports["instance"] = External<RefNapi::Instance>::New(env, data);

  PointerBuffer::Init(env, exports);

  Object smap = Object::New(env);
#define SET_SIZEOF(name, type) smap[#name] = Number::New(env, sizeof(type));
  SET_SIZEOF(int8, int8_t);
  SET_SIZEOF(uint8, uint8_t);
  SET_SIZEOF(int16, int16_t);
  SET_SIZEOF(uint16, uint16_t);
  SET_SIZEOF(int32, int32_t);
  SET_SIZEOF(uint32, uint32_t);
  SET_SIZEOF(int64, int64_t);
  SET_SIZEOF(uint64, uint64_t);
  SET_SIZEOF(float, float);
  SET_SIZEOF(double, double);
  SET_SIZEOF(bool, bool);
  SET_SIZEOF(byte, unsigned char);
  SET_SIZEOF(char, char);
  SET_SIZEOF(uchar, unsigned char);
  SET_SIZEOF(short, short);
  SET_SIZEOF(ushort, unsigned short);
  SET_SIZEOF(int, int);
  SET_SIZEOF(uint, unsigned int);
  SET_SIZEOF(long, long);
  SET_SIZEOF(ulong, unsigned long);
  SET_SIZEOF(longlong, long long);
  SET_SIZEOF(ulonglong, unsigned long long);
  SET_SIZEOF(pointer, char*);
  SET_SIZEOF(size_t, size_t);
  SET_SIZEOF(Object, Reference<Object>);
#undef SET_SIZEOF

  Object amap = Object::New(env);
#define SET_ALIGNOF(name, type) \
  struct s_##name {             \
    type a;                     \
  };                            \
  amap[#name] = Number::New(env, alignof(struct s_##name));
  SET_ALIGNOF(int8, int8_t);
  SET_ALIGNOF(uint8, uint8_t);
  SET_ALIGNOF(int16, int16_t);
  SET_ALIGNOF(uint16, uint16_t);
  SET_ALIGNOF(int32, int32_t);
  SET_ALIGNOF(uint32, uint32_t);
  SET_ALIGNOF(int64, int64_t);
  SET_ALIGNOF(uint64, uint64_t);
  SET_ALIGNOF(float, float);
  SET_ALIGNOF(double, double);
  SET_ALIGNOF(bool, bool);
  SET_ALIGNOF(char, char);
  SET_ALIGNOF(uchar, unsigned char);
  SET_ALIGNOF(short, short);
  SET_ALIGNOF(ushort, unsigned short);
  SET_ALIGNOF(int, int);
  SET_ALIGNOF(uint, unsigned int);
  SET_ALIGNOF(long, long);
  SET_ALIGNOF(ulong, unsigned long);
  SET_ALIGNOF(longlong, long long);
  SET_ALIGNOF(ulonglong, unsigned long long);
  SET_ALIGNOF(pointer, char*);
  SET_ALIGNOF(size_t, size_t);
  SET_ALIGNOF(Object, Reference<Object>);
#undef SET_ALIGNOF

  exports["sizeof"] = smap;
  exports["alignof"] = amap;
  exports["nullptr"] = exports["NULL"] = CreatePointerBuffer(env, nullptr, 0);
  exports["address"] = Function::New(env, Address);
  exports["hexAddress"] = Function::New(env, HexAddress);
  exports["isNull"] = Function::New(env, IsNull);
  exports["isAddress"] = Function::New(env, IsAddress);
  exports["readObject"] = Function::New(env, ReadObject);
  exports["_writeObject"] = Function::New(env, WriteObject);
  exports["readPointer"] = Function::New(env, ReadPointer);
  exports["_writePointer"] = Function::New(env, WritePointer);
  exports["readInt64"] = Function::New(env, ReadInt64);
  exports["writeInt64"] = Function::New(env, WriteInt64);
  exports["readUInt64"] = Function::New(env, ReadUInt64);
  exports["writeUInt64"] = Function::New(env, WriteUInt64);

  exports["readInt32"] = Function::New(env, ReadInt32);
  exports["writeInt32"] = Function::New(env, WriteInt32);

  exports["readUInt32"] = Function::New(env, ReadUInt32);
  exports["readInt8"] = Function::New(env, ReadInt8);
  exports["readUInt8"] = Function::New(env, ReadUInt8);
  exports["readFloat"] = Function::New(env, ReadFloat);
  exports["readDouble"] = Function::New(env, ReadDouble);
  exports["readInt16"] = Function::New(env, ReadInt16);
  exports["readUInt16"] = Function::New(env, ReadUInt16);

  exports["readCString"] = Function::New(env, ReadCString);
  exports["_reinterpret"] = Function::New(env, ReinterpretBuffer);
  exports["_reinterpretUntilZeros"] =
      Function::New(env, ReinterpretBufferUntilZeros);

  return exports;
}

}  // namespace rclnodejs
