// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
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

// tsup configuration for the dual ESM + CommonJS build.
//
// The authored source is native ESM (package.json "type": "module"). External
// consumers that use `require('rclnodejs')` still need a CommonJS build, so
// tsup emits both formats into a FLAT dist/ (depth-1) layout:
//
//   dist/index.js   dist/index.cjs    (exports ".")
//   dist/web.js     dist/web.cjs      (exports "./web")
//   dist/server.js  dist/server.cjs   (exports "./web/server")
//   dist/rosocket.js dist/rosocket.cjs (exports "./rosocket")
//
// The flat depth-1 layout is deliberate: the native loader resolves the
// prebuilt binaries and the `.node` addon via `path.join(__dirname, '..', ...)`
// and `bindings()` walking up to package.json. With every bundle one level
// under the package root, those `..`-relative paths stay correct.

import path from 'path';
import { fileURLToPath } from 'url';
import { defineConfig } from 'tsup';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const DIST = path.join(ROOT, 'dist');

// The generator/parser trees (rosidl_gen/*, rostsd_gen/*) are CommonJS islands
// that resolve their own template and `generated/` paths from their real
// on-disk location. They must NOT be bundled. esbuild would otherwise inline
// them and break `__dirname`-relative template lookups. Keep them external and
// rewrite each specifier to a path that is correct relative to the flat dist/
// output (e.g. `./rosidl_gen/index.cjs` -> `../rosidl_gen/index.cjs`).
const externalizeIslands = {
  name: 'externalize-cjs-islands',
  setup(build) {
    build.onResolve({ filter: /[\\/](rosidl_gen|rostsd_gen)[\\/]/ }, (args) => {
      const abs = path.resolve(args.resolveDir, args.path);
      let rel = path.relative(DIST, abs).split(path.sep).join('/');
      if (!rel.startsWith('.')) {
        rel = './' + rel;
      }
      return { path: rel, external: true };
    });
  },
};

export default defineConfig({
  entry: {
    index: 'index.js',
    web: 'web/index.js',
    server: 'lib/runtime/index.js',
    rosocket: 'rosocket/index.js',
  },
  outDir: 'dist',
  format: ['esm', 'cjs'],
  platform: 'node',
  target: 'node20',
  // One self-contained file per entry/format keeps the dist/ layout flat and
  // the externalized island paths consistent.
  splitting: false,
  sourcemap: true,
  clean: true,
  // Type declarations are handled separately; do not emit here.
  dts: false,
  // Provide import.meta.url in the CJS output (and __dirname in ESM) so the
  // native loader's path resolution works in both formats.
  shims: true,
  esbuildPlugins: [externalizeIslands],
  esbuildOptions(options, context) {
    // Hoist a lone `export default` to `module.exports` in the CJS output so
    // that `const rclnodejs = require('rclnodejs')` keeps working with no
    // `.default` indirection. Only collapses when `default` is the sole export
    // (the root entry); named-only entries such as ./web are left untouched.
    if (context.format === 'cjs') {
      options.footer = {
        js: 'if (module.exports && module.exports.default !== void 0 && Object.keys(module.exports).every(function (k) { return k === "default" || k === "__esModule"; })) { module.exports = module.exports.default; }',
      };
    }
  },
});
