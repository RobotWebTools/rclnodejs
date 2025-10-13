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
const path = require('path');
const { execSync } = require('child_process');
const { detectUbuntuCodename } = require('../lib/utils');

function getRosDistro() {
  return process.env.ROS_DISTRO || null;
}

function checkPrebuiltBinary() {
  const platform = process.platform;
  const arch = process.arch;

  // Only Linux has prebuilt binaries
  if (platform !== 'linux') {
    console.log(
      `Platform ${platform} does not have prebuilt binaries, will build from source`
    );
    return false;
  }

  const ubuntuCodename = detectUbuntuCodename();
  const rosDistro = getRosDistro();

  if (!ubuntuCodename || !rosDistro) {
    console.log(
      'Cannot detect Ubuntu codename or ROS distribution, will build from source'
    );
    return false;
  }

  // Check for the specific prebuilt binary
  const prebuildDir = path.join(
    __dirname,
    '..',
    'prebuilds',
    `${platform}-${arch}`
  );
  const expectedBinary = `${rosDistro}-${ubuntuCodename}-${arch}-rclnodejs.node`;
  const binaryPath = path.join(prebuildDir, expectedBinary);

  if (fs.existsSync(binaryPath)) {
    console.log(`✓ Found prebuilt binary: ${expectedBinary}`);
    console.log(`  Platform: ${platform}, Arch: ${arch}`);
    console.log(`  Ubuntu: ${ubuntuCodename}, ROS: ${rosDistro}`);
    return true;
  }

  console.log(
    `✗ No prebuilt binary found for ${rosDistro}-${ubuntuCodename}-${arch}`
  );

  // List available binaries for debugging
  if (fs.existsSync(prebuildDir)) {
    const availableBinaries = fs
      .readdirSync(prebuildDir)
      .filter((f) => f.endsWith('.node'));
    if (availableBinaries.length > 0) {
      console.log(
        '  Available prebuilt binaries:',
        availableBinaries.join(', ')
      );
    }
  }

  return false;
}

function buildFromSource() {
  console.log('\n=== Building rclnodejs from source ===');
  try {
    execSync('npm run rebuild', {
      stdio: 'inherit',
      cwd: path.join(__dirname, '..'),
      timeout: 600000, // 10 minute timeout
    });
    console.log('✓ Successfully built from source\n');
  } catch (error) {
    console.error('✗ Failed to build from source:', error.message);
    process.exit(1);
  }
}

function main() {
  console.log('\n=== Installing rclnodejs ===\n');

  if (checkPrebuiltBinary()) {
    console.log('✓ Installation complete - using prebuilt binary\n');
  } else {
    console.log('→ Building from source...\n');
    buildFromSource();
  }
}

if (require.main === module) {
  main();
}

module.exports = { checkPrebuiltBinary, buildFromSource };
