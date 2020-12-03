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

const exec = require('child_process').exec;

const cmd = 'wget -nc ';
const cpplintUrl =
  'https://raw.githubusercontent.com/google/styleguide' +
  '/gh-pages/cpplint/cpplint.py';
const root = `${__dirname}/../src/`;
const args = `--extensions=cpp,h,hpp,cc ${root}/*`;

console.log('Downloading the cpplint...');
exec(cmd + cpplintUrl, (err, stdout, stderr) => {
  if (err) {
    console.log(`Downloading failed: ${stderr}`);
  } else {
    console.log('Running the cpplint...');
    exec('python cpplint.py ' + args, (err, stdout, stderr) => {
      console.log(stdout);
      if (err) {
        console.log(stderr);
        throw Error('cpplint failed.');
      }
    });
  }
});
