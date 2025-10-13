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
const { detectUbuntuCodename } = require('../lib/utils');

function tagPrebuilds() {
  const rosDistro = process.env.ROS_DISTRO;
  const ubuntuCodename = detectUbuntuCodename();
  const platform = process.platform;
  const arch = process.arch;

  console.log(
    `Tagging prebuilds with Ubuntu: ${ubuntuCodename || 'unknown'}, ROS2: ${rosDistro || 'unknown'}, Platform: ${platform}, Arch: ${arch}`
  );

  const prebuildDir = path.join(
    __dirname,
    '..',
    'prebuilds',
    `${platform}-${arch}`
  );

  if (!fs.existsSync(prebuildDir)) {
    console.log('No prebuilds directory found, skipping tagging');
    return;
  }

  const files = fs.readdirSync(prebuildDir).filter((f) => f.endsWith('.node'));

  for (const file of files) {
    const filePath = path.join(prebuildDir, file);

    // Create tagged version with format: {ros_distro}-{linux-codename}-{cpu-arch}-rclnodejs.node
    if (rosDistro && ubuntuCodename) {
      const taggedName = `${rosDistro}-${ubuntuCodename}-${arch}-rclnodejs.node`;
      const taggedPath = path.join(prebuildDir, taggedName);
      fs.copyFileSync(filePath, taggedPath);
      console.log(`Created tagged binary: ${taggedName}`);

      // Remove the original generic binary file if it's the basic rclnodejs.node
      if (file === 'rclnodejs.node') {
        fs.unlinkSync(filePath);
        console.log(`Removed generic binary: ${file}`);
      }
    } else {
      console.log(
        `Skipping tagging for ${file} - missing ROS_DISTRO or Ubuntu codename`
      );
    }
  }
}

if (require.main === module) {
  tagPrebuilds();
}

module.exports = { tagPrebuilds };
