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

const fs = require('fs');

/**
 * Detect Ubuntu codename from /etc/os-release
 * @returns {string|null} Ubuntu codename (e.g., 'noble', 'jammy') or null if not detectable
 */
function detectUbuntuCodename() {
  if (process.platform !== 'linux') {
    return null;
  }

  try {
    const osRelease = fs.readFileSync('/etc/os-release', 'utf8');
    const match = osRelease.match(/^VERSION_CODENAME=(.*)$/m);
    return match ? match[1].trim() : null;
  } catch (e) {
    return null;
  }
}

module.exports = {
  detectUbuntuCodename,
};
