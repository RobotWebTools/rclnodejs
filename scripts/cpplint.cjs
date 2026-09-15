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
const path = require('path');

const cmd = 'wget -nc ';
const cpplintUrl =
  'https://raw.githubusercontent.com/cpplint/cpplint/refs/heads/develop/cpplint.py';
const repositoryRoot = path.resolve(__dirname, '..');
const root = path.join(repositoryRoot, 'src');
const args = `--repository="${repositoryRoot}" --filter=-build/include_subdir,-whitespace/indent_namespace --extensions=cpp,h,hpp,cc "${root}"/*`;

console.log('Downloading the cpplint...');
exec(cmd + cpplintUrl, (err, stdout, stderr) => {
  if (err) {
    console.error(`Downloading failed: ${stderr}`);
    process.exitCode = 1;
  } else {
    console.log('Running the cpplint...');
    exec('python3 cpplint.py ' + args, (err, stdout, stderr) => {
      console.log(stdout);
      if (err) {
        console.log(stderr);
        throw Error('cpplint failed.');
      }
    });
  }
});
