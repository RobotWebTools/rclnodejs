// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

const os = require('os');

module.exports = {
  /**
   * Get the python executable on any platform suitable to be handed to *child_process.execFile*.
   * @param {string} py either "python2" or "python3"
   * @return {[string, string[]]} A command to execute directly and additional arguments as an array.
   */
  getPythonExecutable(py) {
    if (os.type() === 'Windows_NT') {
      return ['py', [py === 'python' ? '-2' : '-3']];
    }
    return [py, []];
  },
};
