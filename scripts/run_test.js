// Copyright (c) 2017 Intel Corporation. All rights reserved.

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

const fs = require('fs');
const utils = require('../lib/utils.js');
const Mocha = require('mocha');
const os = require('os');
const path = require('path');

utils
  .remove(path.join(path.dirname(__dirname), 'generated'))
  .then(() => {
    let mocha = new Mocha();
    const testDir = path.join(__dirname, '../test/');
    // eslint-disable-next-line
    const tests = fs.readdirSync(testDir).filter((file) => {
      return file.substr(0, 5) === 'test-';
    });

    // eslint-disable-next-line
    let blocklist = JSON.parse(
      fs.readFileSync(path.join(__dirname, '../test/blocklist.json'), 'utf8')
    );
    let ignoredCases = blocklist[os.type()];

    tests.forEach((test) => {
      if (ignoredCases.indexOf(test) === -1) {
        mocha.addFile(path.join(testDir, test));
      }
    });

    mocha.run(function (failures) {
      process.exitCode = failures ? 1 : 0;
      process.exit(process.exitCode);
    });
  })
  .catch((err) => {
    console.error('Failed to clean generated directory:', err);
    process.exit(1);
  });
