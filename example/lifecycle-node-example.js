// Copyright (c) 2020 Wayne Parrott. All rights reserved.
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

const rclnodejs = require('../index.js');

const NODE_NAME = 'test_node';
const TOPIC = 'test';
const COUNTD_DOWN = 5;

/**
 * This app demonstrates using a LifecycleNode to
 * publish a count down value from 10 - 0. A subscription
 * is created to watch for the counter reaching 0 at which
 * time it will deactivate and shutdown the node.
 */
class App {

  constructor() {
    this._node = null;
    this._publisher = null;
    this._subscriber = null;
    this._timer = null;
    this._StateInterface = null;
  }

  async init() {
    await rclnodejs.init();

    this._count = COUNTD_DOWN;
    this._node = rclnodejs.createLifecycleNode(NODE_NAME);
    this._node.registerOnConfigure((prevState)=>this.onConfigure(prevState));
    this._node.registerOnActivate((prevState)=>this.onActivate(prevState));
    this._node.registerOnDeactivate((prevState)=>this.onDeactivate(prevState));
    this._node.registerOnShutdown((prevState)=>this.onShutdown(prevState));
    this._StateInterface = rclnodejs.createMessage('lifecycle_msgs/msg/State').constructor;

    rclnodejs.spin(this._node);
  }

  start() {
    this._node.configure();
    this._node.activate();
  }

  stop() {
    this._node.deactivate();
    this._node.shutdown();
    rclnodejs.shutdown();
    process.exit(0);
  }

  onConfigure() {
    console.log('Lifecycle: CONFIGURE');
    this._publisher =
      this._node.createLifecyclePublisher('std_msgs/msg/String', TOPIC);
    this._subscriber =
      this._node.createSubscription('std_msgs/msg/String', TOPIC,
        (msg) => {
          let cnt = parseInt(msg.data, 10);
          console.log(`countdown msg: ${cnt}`);
          if (cnt < 1) {
            this.stop();
          }
        });
    return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
  }

  onActivate() {
    console.log('Lifecycle: ACTIVATE');
    this._publisher.activate();
    this._timer = this._node.createTimer(1000, () => {
      this._publisher.publish(`${this._count--}`);
    });
    return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
  }

  onDeactivate() {
    console.log('Lifecycle: DEACTIVATE');
    this._publisher.deactivate();
    if (this._timer) {
      this._timer.cancel();
      this._timer = null;
    }
    return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
  }

  onShutdown(prevState) {
    console.log('Lifecycle: SHUTDOWN');
    let result = rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
    if (prevState.id === this._StateInterface.PRIMARY_STATE) {
      result = this.onDeactivate();
      this._publisher = null;
      this._subscriber = null;
    }
    
    return result;
  }
}

async function main() {
  let app = new App();
  await app.init();
  app.start();
}

main();


