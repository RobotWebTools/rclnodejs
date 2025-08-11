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

'use strict';

const fse = require('fs-extra');
const path = require('path');

function getGeneratedRoot() {
  let generatedRoot = path.join(__dirname, '../generated/');
  if (process.env.GENERATED_MSG_PATH) {
    if (fse.pathExistsSync(process.env.GENERATED_MSG_PATH)) {
      generatedRoot = path.join(process.env.GENERATED_MSG_PATH, '/generated/');
      console.log(generatedRoot);
    } else {
      console.log(
        `Warning: GENERATED_MSG_PATH is set to '${process.env.GENERATED_MSG_PATH}' but the parent directory does not exist. Using default path instead.`
      );
    }
  }
  return generatedRoot;
}

module.exports = { getGeneratedRoot };
