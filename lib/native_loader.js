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

import fs from 'fs';
import path from 'path';
import childProcess from 'child_process';
import { fileURLToPath } from 'url';
import { createRequire } from 'module';
import { NativeError } from './errors.js';
import bindings from 'bindings';
import createDebug from 'debug';
import {
  detectPrebuildRuntime,
  getTaggedPrebuildFilename,
} from './prebuilds.js';
import { detectUbuntuCodename } from './utils.js';

const require = createRequire(import.meta.url);
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const debug = createDebug('rclnodejs');

let nativeModule = null;

// Simplified loader: only use prebuilt binaries with exact Ubuntu/ROS2/arch match
// Note: Prebuilt binaries are only supported on Linux (Ubuntu) platform
function customFallbackLoader() {
  // Prebuilt binaries are only for Linux platform
  if (process.platform !== 'linux') {
    debug('Prebuilt binaries are only supported on Linux platform');
    return null;
  }

  const rosDistro = process.env.ROS_DISTRO;
  const arch = process.arch;
  const runtime = detectPrebuildRuntime();
  const ubuntuCodename = detectUbuntuCodename();

  // Require all three components for exact match
  if (!rosDistro || !ubuntuCodename || !arch) {
    debug(
      `Missing environment info - ROS: ${rosDistro}, Ubuntu: ${ubuntuCodename}, Arch: ${arch}`
    );
    return null;
  }

  const prebuildDir = path.join(
    __dirname,
    '..',
    'prebuilds',
    `${process.platform}-${arch}`
  );

  if (!fs.existsSync(prebuildDir)) {
    debug('No prebuilds directory found');
    return null;
  }

  try {
    const candidate = getTaggedPrebuildFilename({
      rosDistro,
      ubuntuCodename,
      arch,
      runtime,
    });
    const candidatePath = path.join(prebuildDir, candidate);

    if (fs.existsSync(candidatePath)) {
      debug(`Found ${runtime} prebuilt binary: ${candidate}`);
      return require(candidatePath);
    }

    debug(
      `No matching ${runtime} prebuilt binary found for ${rosDistro}-${ubuntuCodename}-${arch}`
    );
    return null;
  } catch (e) {
    debug('Error in simplified prebuilt loader:', e.message);
  }

  return null;
}

// Simplified prebuilt binary loader: exact match or build from source
function loadNativeAddon() {
  if (nativeModule) {
    return nativeModule;
  }

  // Environment variable to force building from source
  if (process.env.RCLNODEJS_FORCE_BUILD === '1') {
    debug('Forcing build from source (RCLNODEJS_FORCE_BUILD=1)');

    // Trigger actual compilation
    try {
      debug('Running forced node-gyp rebuild...');
      childProcess.execSync('npm run rebuild', {
        stdio: 'inherit',
        cwd: path.join(__dirname, '..'),
        timeout: 300000, // 5 minute timeout
      });

      // Load the newly built binary
      nativeModule = bindings('rclnodejs');
      debug('Successfully force compiled and loaded from source');
      return nativeModule;
    } catch (compileError) {
      debug('Forced compilation failed:', compileError.message);
      throw new NativeError(
        `Failed to force build rclnodejs from source: ${compileError.message}`,
        'Forced compilation',
        { cause: compileError }
      );
    }
  }

  const rosDistro = process.env.ROS_DISTRO;
  const runtime = detectPrebuildRuntime();
  const ubuntuCodename = detectUbuntuCodename();

  debug(
    `Platform: ${process.platform}, Arch: ${process.arch}, Runtime: ${runtime}, Ubuntu: ${ubuntuCodename || 'unknown'}, ROS: ${rosDistro || 'unknown'}`
  );

  // Prebuilt binaries are only supported on Linux (Ubuntu)
  if (process.platform === 'linux') {
    // Try exact match prebuilt binary first
    try {
      const prebuiltModule = customFallbackLoader();
      if (prebuiltModule) {
        nativeModule = prebuiltModule;
        return nativeModule;
      }
    } catch (e) {
      debug('Exact match prebuilt loading failed:', e.message);
    }

    debug(
      'No exact match prebuilt binary found, falling back to build from source'
    );
  } else {
    debug(
      `Platform ${process.platform} does not support prebuilt binaries, will try existing build or compile from source`
    );
  }

  try {
    // Try to find existing built binary first (works on all platforms)
    // The 'bindings' module will search standard locations like:
    // - build/Release/rclnodejs.node
    // - build/Debug/rclnodejs.node
    // - compiled/{node_version}/{platform}/{arch}/rclnodejs.node
    // etc.
    nativeModule = bindings('rclnodejs');
    debug('Found and loaded existing built binary');
    return nativeModule;
  } catch {
    debug('No existing built binary found, triggering compilation...');

    // Trigger actual compilation
    try {
      debug('Running node-gyp rebuild...');
      childProcess.execSync('npm run rebuild', {
        stdio: 'inherit',
        cwd: path.join(__dirname, '..'),
        timeout: 300000, // 5 minute timeout
      });

      // Try to load the newly built binary
      nativeModule = bindings('rclnodejs');
      debug('Successfully compiled and loaded from source');
      return nativeModule;
    } catch (compileError) {
      debug('Compilation failed:', compileError.message);
      throw new NativeError(
        `Failed to build rclnodejs from source: ${compileError.message}`,
        'Compilation',
        { cause: compileError }
      );
    }
  }
}

const addon = loadNativeAddon();

// Export internal functions for testing purposes
if (process.env.NODE_ENV === 'test') {
  addon.TestHelpers = {
    customFallbackLoader,
    loadNativeAddon,
  };
}

export default addon;
