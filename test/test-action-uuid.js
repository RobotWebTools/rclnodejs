// Copyright (c) 2022 Wayne Parrott. All rights reserved.
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

const assert = require('assert');
const rclnodejs = require('../index.js');

describe('ActionUuid tests', () => {
  before(function () {
    // need to ensure msgs have been generated before running
    return rclnodejs.init();
  });

  after(function () {
    // need to ensure msgs have been generated before running
    return rclnodejs.shutdownAll();
  });

  it('constructor - defaults', () => {
    let uuid = new rclnodejs.ActionUuid();
    assert.ok(uuid);
    assert.ok(uuid._bytes instanceof Uint8Array);
  });

  it('constructor - custom bytes', () => {
    let bytes = Uint8Array.from([0, 1]);
    let uuid = new rclnodejs.ActionUuid(bytes);
    assert.ok(uuid);
    assert.deepEqual(uuid._bytes, bytes);
  });

  it('static random constructor', () => {
    let uuid = rclnodejs.ActionUuid.random();
    assert.ok(uuid);
    assert.ok(uuid._bytes instanceof Uint8Array);
  });

  it('static fromBytes() constructor', () => {
    let bytes = Uint8Array.from([0, 1]);
    let uuid = rclnodejs.ActionUuid.fromBytes(bytes);
    assert.ok(uuid);
    assert.deepEqual(uuid._bytes, bytes);
  });

  it('bytes getter', () => {
    let bytes = Uint8Array.from([0, 1]);
    let uuid = new rclnodejs.ActionUuid(bytes);
    assert.ok(uuid);
    assert.deepEqual(uuid.bytes, bytes);
  });

  it('toString', () => {
    let bytes = Uint8Array.from([0, 1]);
    let uuid = new rclnodejs.ActionUuid(bytes);
    assert.ok(uuid);
    assert.equal(uuid.toString(), '0,1');
  });

  it('no duplicates', () => {
    let uuid1 = new rclnodejs.ActionUuid();
    let uuid2 = new rclnodejs.ActionUuid();
    assert.notDeepEqual(uuid1.bytes, uuid2.bytes);
  });

  it('toMsg', () => {
    let uuid = new rclnodejs.ActionUuid();
    let msg = uuid.toMessage();
    assert.ok(msg);
    assert.deepEqual(uuid._bytes, msg.uuid);
  });

  it('fromMsg', () => {
    let uuid1 = new rclnodejs.ActionUuid();
    let msg = uuid1.toMessage();
    let uuid2 = rclnodejs.ActionUuid.fromMessage(msg);

    assert.ok(uuid2);
    assert.deepEqual(uuid1.bytes, uuid2.bytes);
  });
});
