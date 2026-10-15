// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import path from 'node:path';
import { setTimeout as delay } from 'node:timers/promises';

const require = createRequire(import.meta.url);
const format = process.argv[2];
assert.ok(format === 'esm' || format === 'cjs');
const load = (name) => (format === 'cjs' ? require(name) : import(name));
const entry = await load('rclnodejs');
const rcl = format === 'esm' ? entry.default : entry;
assert.equal(typeof rcl.init, 'function');
assert.equal(rcl.default, undefined);

const { connect } = await load('rclnodejs/web');
const { createRuntime, WebSocketTransport, HttpTransport } = await load(
  'rclnodejs/web/server'
);
const { startRosocket } = await load('rclnodejs/rosocket');
assert.equal(typeof startRosocket, 'function');

const packageRoot = path.dirname(require.resolve('rclnodejs/package.json'));
assert.ok(
  Object.keys(require.cache).includes(path.join(packageRoot, process.argv[3])),
  'The native addon must be loaded from the installed package'
);

await rcl.init();
let runtime;
const clients = [];
try {
  const Message = rcl.require('std_msgs/msg/String');
  assert.equal(new Message({ data: 'packed' }).data, 'packed');
  await rcl.regenerateAll();
  const Custom = rcl.require('tsdown_test_msgs/msg/Envelope');
  const custom = new Custom({
    header: { frame_id: 'packed_frame' },
    label: 'custom interface',
    serial: 9007199254740993n,
  });
  assert.equal(custom.header.frame_id, 'packed_frame');
  assert.equal(custom.label, 'custom interface');
  assert.equal(custom.serial, 9007199254740993n);
  const Fibonacci = rcl.require('example_interfaces/action/Fibonacci');
  const scope = `/package_${process.pid}`;
  const node = rcl.createNode(`package_${format}_${process.pid}`);
  node.createService(
    'example_interfaces/srv/AddTwoInts',
    `${scope}/add`,
    (request, response) => {
      const reply = response.template;
      reply.sum = request.a + request.b;
      response.send(reply);
    }
  );
  new rcl.ActionServer(
    node,
    'example_interfaces/action/Fibonacci',
    `${scope}/action`,
    async (goal) => {
      goal.publishFeedback(new Fibonacci.Feedback({ sequence: [0, 1] }));
      await delay(50);
      goal.succeed();
      return new Fibonacci.Result({ sequence: [0, 1, 1] });
    }
  );
  rcl.spin(node);
  runtime = createRuntime({
    node,
    transports: [
      new WebSocketTransport({ host: '127.0.0.1', port: 0 }),
      new HttpTransport({ host: '127.0.0.1', port: 0 }),
    ],
  });
  runtime.expose({
    call: { [`${scope}/add`]: 'example_interfaces/srv/AddTwoInts' },
    publish: { [`${scope}/chatter`]: 'std_msgs/msg/String' },
    subscribe: { [`${scope}/chatter`]: 'std_msgs/msg/String' },
    action: { [`${scope}/action`]: 'example_interfaces/action/Fibonacci' },
  });
  await runtime.start();
  clients.push(
    await connect(`ws://127.0.0.1:${runtime.transports[0].port}/capability`)
  );
  clients.push(
    await connect({ http: `http://127.0.0.1:${runtime.transports[1].port}` })
  );

  let received;
  const subscription = await clients[0].subscribe(
    `${scope}/chatter`,
    (message) => {
      received = message.data;
    }
  );
  const deadline = Date.now() + 5000;
  while (received === undefined && Date.now() < deadline) {
    await clients[1].publish(`${scope}/chatter`, { data: 'packed message' });
    await delay(100);
  }
  assert.equal(received, 'packed message');
  await subscription.close();

  for (const client of clients) {
    assert.deepEqual(await client.call(`${scope}/add`, { a: '2n', b: '40n' }), {
      sum: '42n',
    });
    const feedback = [];
    const goal = await client.action(
      `${scope}/action`,
      { order: 3 },
      { onFeedback: (message) => feedback.push(message.sequence) }
    );
    assert.deepEqual(await goal.result, { sequence: [0, 1, 1] });
    assert.equal(goal.status, 'succeeded');
    assert.deepEqual(feedback, [[0, 1]]);
  }
  console.log(
    `${format}: installed exports, native loading and ROS round trips passed`
  );
} finally {
  for (const client of clients) await client.close();
  if (runtime) await runtime.stop();
  rcl.shutdown();
}
