// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Action capability dispatch coverage: raw wire-protocol frames (mirroring
// test-runtime.js) plus SDK-level (`rclnodejs/web`) round-trips over both
// the WebSocket and HTTP transports.

import assert from 'assert';
import http from 'node:http';
import WebSocket from 'ws';
import rclnodejs from '../index.js';
import {
  createRuntime,
  WebSocketTransport,
  HttpTransport,
} from '../lib/runtime/index.js';
import * as assertUtils from './utils.js';

// `web/` is ESM; dynamic import() defers loading the browser SDK until the test starts.
let connect;
before(async function () {
  ({ connect } = await import('../web/index.js'));
});

describe('Action capability dispatch', function () {
  this.timeout(60 * 1000);

  const fibonacci = 'example_interfaces/action/Fibonacci';
  let Fibonacci;
  let node;
  let runtime;
  let server;
  let wsUrl;
  let httpUrl;

  function waitOpen(ws) {
    return new Promise((resolve, reject) => {
      ws.once('open', resolve);
      ws.once('error', reject);
    });
  }

  function waitFrame(ws, predicate) {
    return new Promise((resolve) => {
      const onMsg = (data) => {
        const frame = JSON.parse(data.toString('utf8'));
        if (predicate(frame)) {
          ws.off('message', onMsg);
          resolve(frame);
        }
      };
      ws.on('message', onMsg);
    });
  }

  // Long enough to leave a window for feedback/cancel to land before the
  // goal finishes, short enough to keep the suite fast.
  async function executeCallback(goalHandle) {
    const feedback = new Fibonacci.Feedback();
    feedback.sequence = [1, 1];
    goalHandle.publishFeedback(feedback);
    await assertUtils.createDelay(50);
    if (goalHandle.isCancelRequested) {
      goalHandle.canceled();
      return new Fibonacci.Result();
    }
    await assertUtils.createDelay(50);
    goalHandle.succeed();
    const result = new Fibonacci.Result();
    result.sequence = [1, 1, 2, 3];
    return result;
  }

  function cancelCallback() {
    return rclnodejs.CancelResponse.ACCEPT;
  }

  before(async function () {
    await rclnodejs.init();
    Fibonacci = rclnodejs.require(fibonacci);
    node = rclnodejs.createNode('action_dispatch_test_node');
    rclnodejs.spin(node);

    server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      '/fibonacci',
      executeCallback,
      null,
      null,
      cancelCallback
    );

    runtime = createRuntime({
      node,
      transports: [
        new WebSocketTransport({ port: 0, host: '127.0.0.1' }),
        new HttpTransport({ port: 0, host: '127.0.0.1' }),
      ],
    });
    runtime.expose({ action: { '/fibonacci': fibonacci } });
    await runtime.start();
    wsUrl = `ws://127.0.0.1:${runtime.transports[0].port}/capability`;
    httpUrl = `http://127.0.0.1:${runtime.transports[1].port}`;
  });

  after(async function () {
    if (server) server.destroy();
    if (runtime) await runtime.stop();
    rclnodejs.shutdown();
  });

  describe('wire protocol (WebSocket)', function () {
    it('rejects send_goal against an unexposed action with code:not_exposed', async function () {
      const ws = new WebSocket(wsUrl);
      await waitOpen(ws);
      const replyP = waitFrame(ws, (f) => f.id === 'g1');
      ws.send(
        JSON.stringify({
          id: 'g1',
          kind: 'action',
          op: 'send_goal',
          capability: '/no_such_action',
          payload: {},
        })
      );
      const reply = await replyP;
      assert.strictEqual(reply.ok, false);
      assert.strictEqual(reply.code, 'not_exposed');
      ws.close();
    });

    it('accepts a goal, streams feedback, then a terminal result', async function () {
      const ws = new WebSocket(wsUrl);
      await waitOpen(ws);
      const ackP = waitFrame(ws, (f) => f.id === 'g2');
      const feedbackP = waitFrame(
        ws,
        (f) => f.event === 'feedback' && f.goalId === 'g2'
      );
      const resultP = waitFrame(
        ws,
        (f) => f.event === 'result' && f.goalId === 'g2'
      );
      ws.send(
        JSON.stringify({
          id: 'g2',
          kind: 'action',
          op: 'send_goal',
          capability: '/fibonacci',
          payload: { order: 5 },
        })
      );
      const ack = await ackP;
      assert.strictEqual(ack.ok, true);
      assert.strictEqual(ack.payload.accepted, true);

      const feedback = await feedbackP;
      assert.deepStrictEqual(feedback.payload.sequence, [1, 1]);

      const result = await resultP;
      assert.strictEqual(result.status, 'succeeded');
      assert.deepStrictEqual(result.payload.sequence, [1, 1, 2, 3]);
      ws.close();
    });

    it('cancels an in-flight goal', async function () {
      const ws = new WebSocket(wsUrl);
      await waitOpen(ws);
      const ackP = waitFrame(ws, (f) => f.id === 'g3');
      ws.send(
        JSON.stringify({
          id: 'g3',
          kind: 'action',
          op: 'send_goal',
          capability: '/fibonacci',
          payload: { order: 5 },
        })
      );
      await ackP;

      const resultP = waitFrame(
        ws,
        (f) => f.event === 'result' && f.goalId === 'g3'
      );
      const cancelAckP = waitFrame(ws, (f) => f.id === 'c1');
      ws.send(
        JSON.stringify({
          id: 'c1',
          kind: 'action',
          op: 'cancel',
          goalId: 'g3',
        })
      );
      const cancelAck = await cancelAckP;
      assert.strictEqual(cancelAck.ok, true);

      const result = await resultP;
      assert.strictEqual(result.status, 'canceled');
      ws.close();
    });

    it('rejects cancel of an unknown goal with code:unknown_goal_id', async function () {
      const ws = new WebSocket(wsUrl);
      await waitOpen(ws);
      const replyP = waitFrame(ws, (f) => f.id === 'c2');
      ws.send(
        JSON.stringify({
          id: 'c2',
          kind: 'action',
          op: 'cancel',
          goalId: 'no-such-goal',
        })
      );
      const reply = await replyP;
      assert.strictEqual(reply.ok, false);
      assert.strictEqual(reply.code, 'unknown_goal_id');
      ws.close();
    });

    it('rejects a duplicate goal id while the first send is pending', async function () {
      const ws = new WebSocket(wsUrl);
      await waitOpen(ws);
      const duplicateP = waitFrame(
        ws,
        (f) => f.id === 'duplicate' && f.code === 'duplicate_id'
      );
      const frame = JSON.stringify({
        id: 'duplicate',
        kind: 'action',
        op: 'send_goal',
        capability: '/fibonacci',
        payload: { order: 5 },
      });
      ws.send(frame);
      ws.send(frame);
      const duplicate = await duplicateP;
      assert.strictEqual(duplicate.ok, false);
      ws.close();
    });

    it('survives disconnect while send_goal is pending', async function () {
      const ws = new WebSocket(wsUrl);
      await waitOpen(ws);
      ws.send(
        JSON.stringify({
          id: 'disconnect',
          kind: 'action',
          op: 'send_goal',
          capability: '/fibonacci',
          payload: { order: 5 },
        })
      );
      ws.close();
      await assertUtils.createDelay(200);

      const probe = new WebSocket(wsUrl);
      await waitOpen(probe);
      probe.close();
    });
  });

  describe('SDK (rclnodejs/web)', function () {
    it('sends a goal over WebSocket and awaits the result, with feedback', async function () {
      const ros = await connect(wsUrl);
      try {
        const feedbacks = [];
        const goal = await ros.action(
          '/fibonacci',
          { order: 5 },
          { onFeedback: (fb) => feedbacks.push(fb) }
        );
        const result = await goal.result;
        assert.deepStrictEqual(result.sequence, [1, 1, 2, 3]);
        assert.strictEqual(feedbacks.length, 1);
        assert.deepStrictEqual(feedbacks[0].sequence, [1, 1]);
      } finally {
        await ros.close();
      }
    });

    it('cancels a goal over WebSocket', async function () {
      const ros = await connect(wsUrl);
      try {
        const goal = await ros.action('/fibonacci', { order: 5 });
        await assertUtils.createDelay(20);
        await goal.cancel();
        const result = await goal.result;
        assert.deepStrictEqual(result, { sequence: [] });
      } finally {
        await ros.close();
      }
    });

    it('sends a goal over HTTP (SSE) and awaits the result, with feedback', async function () {
      const ros = await connect(httpUrl);
      try {
        const feedbacks = [];
        const goal = await ros.action(
          '/fibonacci',
          { order: 5 },
          { onFeedback: (fb) => feedbacks.push(fb) }
        );
        const result = await goal.result;
        assert.deepStrictEqual(result.sequence, [1, 1, 2, 3]);
        assert.strictEqual(feedbacks.length, 1);
        assert.deepStrictEqual(feedbacks[0].sequence, [1, 1]);
      } finally {
        await ros.close();
      }
    });

    it('rejects cancel over HTTP with code:unsupported_kind', async function () {
      const ros = await connect(httpUrl);
      try {
        const goal = await ros.action('/fibonacci', { order: 5 });
        await assert.rejects(goal.cancel(), (err) => {
          assert.strictEqual(err.code, 'unsupported_kind');
          return true;
        });
        await goal.result;
      } finally {
        await ros.close();
      }
    });

    it('rejects when an HTTP action stream ends without a result', async function () {
      const truncatedServer = http.createServer((req, res) => {
        res.writeHead(200, { 'content-type': 'text/event-stream' });
        res.end('event: accepted\ndata: {}\n\n');
      });
      await new Promise((resolve) =>
        truncatedServer.listen(0, '127.0.0.1', resolve)
      );
      const address = truncatedServer.address();
      const ros = await connect(`http://127.0.0.1:${address.port}`);
      try {
        const goal = await ros.action('/fibonacci', { order: 5 });
        await assert.rejects(goal.result, (err) => {
          assert.strictEqual(err.code, 'connection_lost');
          return true;
        });
      } finally {
        await ros.close();
        await new Promise((resolve, reject) =>
          truncatedServer.close((err) => (err ? reject(err) : resolve()))
        );
      }
    });
  });
});
