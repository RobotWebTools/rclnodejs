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

import path from 'path';
import { fileURLToPath } from 'url';
import { defineConfig } from 'tsdown';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const DIST = path.join(ROOT, 'dist');

// Generators resolve templates and generated interfaces from their on-disk
// locations. Keep them external and render paths relative to each output file.
/** @type {import('tsdown').Rolldown.Plugin} */
const externalizeIslands = {
  name: 'externalize-cjs-islands',
  resolveId(source, importer) {
    if (!/[\\/](rosidl_gen|rostsd_gen)[\\/]/.test(source)) return null;
    const absolute = path.resolve(
      importer ? path.dirname(importer) : ROOT,
      source
    );
    const relative = path.relative(DIST, absolute).split(path.sep).join('/');
    return {
      id: relative.startsWith('.') ? relative : `./${relative}`,
      external: true,
    };
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
  sourcemap: true,
  clean: true,
  // Public declarations and ROS interface types are maintained separately.
  dts: false,
  shims: true,
  cjsDefault: true,
  plugins: [externalizeIslands],
  outputOptions(options, format) {
    const extension = format === 'cjs' ? 'cjs' : 'js';
    // Entry names match package exports. Shared chunks must also stay one
    // level below the package root for native addon and interface loading.
    return {
      ...options,
      entryFileNames: `[name].${extension}`,
      chunkFileNames: `[name]-[hash].${extension}`,
    };
  },
});
