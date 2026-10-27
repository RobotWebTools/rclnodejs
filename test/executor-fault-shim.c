#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef __GLIBC__
#include <link.h>
#include <string.h>
#endif

static _Atomic int trigger_failures;

static void* lookup(const char* library, const char* name) {
  void* handle = dlopen(library, RTLD_NOLOAD | RTLD_NOW);
  if (!handle) abort();
  void* symbol = dlsym(handle, name);
  if (!symbol) abort();
  dlclose(handle);
  return symbol;
}

static void record_marker(char marker) {
  const char* path = getenv("RCLNODEJS_TEST_EXECUTOR_MARKER");
  if (!path) return;
  int descriptor = open(path, O_WRONLY | O_CREAT | O_APPEND, 0600);
  if (descriptor < 0 || write(descriptor, &marker, 1) != 1) abort();
  close(descriptor);
}

static void set_error(const char* message) {
  typedef void (*set_error_fn)(const char*, const char*, size_t);
  ((set_error_fn)lookup("librcutils.so", "rcutils_set_error_state"))(
      message, "executor-fault-shim", 1);
}

#ifdef __GLIBC__
static int (*real_async_init)(void*, void*, void*);
static int (*real_thread_create)(void*, void*, void*);
static int (*real_thread_join)(void*);
static _Atomic int uv_failures;

unsigned int la_version(unsigned int version) {
  return version >= LAV_CURRENT ? LAV_CURRENT : version;
}

unsigned int la_objopen(struct link_map* map, Lmid_t namespace_id,
                        uintptr_t* cookie) {
  (void)namespace_id;
  *cookie = (uintptr_t)map;
  return LA_FLG_BINDTO | LA_FLG_BINDFROM;
}

static int fail_uv_once(const char* target, char marker) {
  const char* selected = getenv("RCLNODEJS_TEST_FAIL_UV");
  if (selected && strcmp(selected, target) == 0 &&
      atomic_fetch_add(&uv_failures, 1) == 0) {
    record_marker(marker);
    return 1;
  }
  return 0;
}

static int audit_async_init(void* loop, void* handle, void* callback) {
  if (fail_uv_once("async", 'A')) return -12;
  return real_async_init(loop, handle, callback);
}

static int audit_thread_create(void* thread, void* entry, void* argument) {
  if (fail_uv_once("create", 'C')) return -12;
  return real_thread_create(thread, entry, argument);
}

static int audit_thread_join(void* thread) {
  if (fail_uv_once("join", 'J')) return -12;
  return real_thread_join(thread);
}

uintptr_t la_symbind64(Elf64_Sym* symbol, unsigned int index,
                       uintptr_t* refcookie, uintptr_t* defcookie,
                       unsigned int* flags, const char* name) {
  (void)index;
  (void)defcookie;
  (void)flags;
  struct link_map* referrer = (struct link_map*)*refcookie;
  if (!referrer || !strstr(referrer->l_name, "rclnodejs.node")) {
    return symbol->st_value;
  }
  if (strcmp(name, "uv_async_init") == 0) {
    real_async_init = (void*)symbol->st_value;
    return (uintptr_t)audit_async_init;
  }
  if (strcmp(name, "uv_thread_create") == 0) {
    real_thread_create = (void*)symbol->st_value;
    return (uintptr_t)audit_thread_create;
  }
  if (strcmp(name, "uv_thread_join") == 0) {
    real_thread_join = (void*)symbol->st_value;
    return (uintptr_t)audit_thread_join;
  }
  return symbol->st_value;
}
#endif

int rcl_wait(void* wait_set, int64_t timeout) {
  typedef int (*wait_fn)(void*, int64_t);
  record_marker('W');
  return ((wait_fn)lookup("librcl.so", "rcl_wait"))(wait_set, timeout);
}

int rcl_trigger_guard_condition(const void* guard) {
  typedef int (*trigger_fn)(const void*);
  if (getenv("RCLNODEJS_TEST_FAIL_TRIGGER") &&
      atomic_fetch_add(&trigger_failures, 1) == 0) {
    set_error("executor interrupt failure");
    return 1;
  }
  return ((trigger_fn)lookup("librcl.so", "rcl_trigger_guard_condition"))(
      guard);
}