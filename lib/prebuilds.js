// Copyright (c) 2026, The Robot Web Tools Contributors
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

const PREBUILD_PACKAGE_NAME = 'rclnodejs';
const SUPPORTED_PREBUILD_RUNTIMES = new Set(['node', 'electron']);

function detectPrebuildRuntime() {
  if (process.env.npm_config_runtime === 'electron') {
    return 'electron';
  }

  return process.versions.electron ? 'electron' : 'node';
}

function getTaggedPrebuildFilename({
  rosDistro,
  ubuntuCodename,
  arch,
  runtime,
}) {
  return `${rosDistro}-${ubuntuCodename}-${arch}-${runtime}-${PREBUILD_PACKAGE_NAME}.node`;
}

function getRuntimeFromGeneratedPrebuild(fileName) {
  const runtime = fileName.split('.')[0];
  return SUPPORTED_PREBUILD_RUNTIMES.has(runtime) ? runtime : null;
}

export {
  detectPrebuildRuntime,
  getRuntimeFromGeneratedPrebuild,
  getTaggedPrebuildFilename,
  PREBUILD_PACKAGE_NAME,
};
