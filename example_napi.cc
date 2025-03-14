#include <node_api.h>

// ...existing code...

// N-API initialization function
napi_value Init(napi_env env, napi_value exports) {
  // ...existing code...
  return exports;
}

// ...existing code...

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)
