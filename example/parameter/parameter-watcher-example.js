// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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

'use strict';

const rclnodejs = require('../../index.js');

async function main() {
  await rclnodejs.init();

  const node = rclnodejs.createNode('watcher_node');

  const watcher = node.createParameterWatcher('turtlesim', [
    'background_r',
    'background_g',
  ]);

  watcher.on('change', (params) => {
    params.forEach((p) => {
      console.log(`${p.name} changed to ${p.value.integer_value}`);
    });
  });

  try {
    const available = await watcher.start(10000);

    if (!available) {
      console.log('Turtlesim node not available. Please run:');
      console.log('  ros2 run turtlesim turtlesim_node');
      return;
    }

    console.log('Watching:', watcher.watchedParameters);

    watcher.addParameter('background_b');
    console.log('Added background_b. Now watching:', watcher.watchedParameters);

    watcher.removeParameter('background_g');
    console.log(
      'Removed background_g. Now watching:',
      watcher.watchedParameters
    );

    rclnodejs.spin(node);
  } catch (error) {
    console.error('Error:', error.message);
    node.destroy();
    rclnodejs.shutdown();
  }
}

main();
