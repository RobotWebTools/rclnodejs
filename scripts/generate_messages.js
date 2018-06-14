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

const generator = require('../rosidl_gen/generator.js');

console.log('Start to generate the JavaScript messages...');
generator.generateAll(true).then(() => {
  console.log('Generation is done.');
}).catch((e) => {
  console.log(`Caught error: ${e}`);
});
