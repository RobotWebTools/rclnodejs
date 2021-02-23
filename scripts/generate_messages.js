#!/usr/bin/env node

/* eslint-disable camelcase */
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const generator = require('../rosidl_gen/index.js');
const tsdGenerator = require('../rostsd_gen/index.js');

async function main() {
  console.log('Start generation of ROS2 JavaScript messages...');

  try {
    await generator.generateAll(true);
    await tsdGenerator.generateAll(); // create interfaces.d.ts
    console.log('Generation complete.');
  } catch (e) {
    console.log(`Caught error: ${e}`);
  }
}

main();
