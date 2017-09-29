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
const Mocha = require('mocha');
const os = require('os');
const path = require('path');

let mocha = new Mocha();
const testDir = path.join(__dirname, '../test/');
// eslint-disable-next-line
const tests = fs.readdirSync(testDir).filter(file => {
  return file.substr(0, 5) === 'test-';});

const blacklistWindows = ['test-array.js', 'test-cross-lang.js', 'test-msg-type-py-node.js', 'test-message-type.js'];
const blacklistMac = ['test-interactive.js'];
const blacklistLinux = [];
let blacklist = [];

if (os.type() === 'Windows_NT') {
  blacklist = blacklistWindows;
} else if (os.type() === 'Darwin') {
  blacklist = blacklistMac;
} else {
  blacklist = blacklistLinux;
}

tests.forEach(test => {
  if (blacklist.indexOf(test) === -1) {
    mocha.addFile(path.join(testDir, test));
  }
});

mocha.run(function(failures) {
  process.on('exit', () => {
    process.exit(failures);
  });
});
