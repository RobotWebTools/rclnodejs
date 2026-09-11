// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Action protocol and SDK tests over WebSocket and HTTP/SSE.

import assert from 'assert';
import { once } from 'node:events';
import http from 'node:http';
import WebSocket, { WebSocketServer } from 'ws';
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
        assert.strictEqual(goal.status, 'succeeded');
        assert.strictEqual(feedbacks.length, 1);
        assert.deepStrictEqual(feedbacks[0].sequence, [1, 1]);
      } finally {
        await ros.close();
      }
    });

    it('treats null and undefined action options as omitted', async function () {
      const ros = await connect(wsUrl);
      try {
        for (const options of [null, undefined]) {
          const goal = await ros.action('/fibonacci', { order: 5 }, options);
          assert.deepStrictEqual(await goal.result, { sequence: [1, 1, 2, 3] });
          assert.strictEqual(goal.status, 'succeeded');
        }
      } finally {
        await ros.close();
      }
    });

    it('rejects invalid feedback callbacks before opening the lazy WebSocket', async function () {
      const httpUrl = new URL(wsUrl);
      httpUrl.protocol = 'http:';
      httpUrl.pathname = '/';
      const ros = await connect(httpUrl.href);
      try {
        for (const onFeedback of [null, false, 0, 1, 'feedback', {}, []]) {
          await assert.rejects(
            ros.action('/fibonacci', { order: 5 }, { onFeedback }),
            {
              name: 'TypeError',
              message:
                'action(capability, payload, options): onFeedback must be a function',
            }
          );
          assert.strictEqual(ros._ws, null);
          assert.strictEqual(ros._wsConnect, null);
        }
      } finally {
        await ros.close();
      }
    });

    it('isolates errors thrown by valid feedback callbacks', async function () {
      const ros = await connect(wsUrl);
      let feedbackCount = 0;
      try {
        const goal = await ros.action(
          '/fibonacci',
          { order: 5 },
          {
            onFeedback() {
              feedbackCount++;
              throw new Error('feedback callback failed');
            },
          }
        );
        assert.deepStrictEqual(await goal.result, { sequence: [1, 1, 2, 3] });
        assert.strictEqual(goal.status, 'succeeded');
        assert.strictEqual(feedbackCount, 1);
      } finally {
        await ros.close();
      }
    });

    it('does not open the lazy WebSocket during or after client close', async function () {
      const httpUrl = new URL(wsUrl);
      httpUrl.protocol = 'http:';
      httpUrl.pathname = '/';
      const ros = await connect(httpUrl.href);
      const closing = ros.close();
      try {
        await assert.rejects(
          ros.action('/fibonacci', { order: 5 }),
          /connection closed/
        );
        await closing;
        await assert.rejects(
          ros.action('/fibonacci', { order: 5 }),
          /connection closed/
        );
        await assert.rejects(
          ros.subscribe('/feedback', () => {}),
          /connection closed/
        );
        await assert.rejects(ros.connect(), /connection closed/);
        assert.strictEqual(ros._ws, null);
        assert.strictEqual(ros._wsConnect, null);
      } finally {
        await ros.close();
      }
    });

    it('rejects actions during and after WebSocket close without tracking goals', async function () {
      const ros = await connect(wsUrl);
      const closing = ros.close();
      try {
        await assert.rejects(
          ros.action('/fibonacci', { order: 5 }),
          /connection closed/
        );
        await closing;
        await assert.rejects(
          ros.action('/fibonacci', { order: 5 }),
          /connection closed/
        );
        assert.strictEqual(ros._ws._pending.size, 0);
        assert.strictEqual(ros._ws._goals.size, 0);
      } finally {
        await closing;
      }
    });

    it('cancels a goal over WebSocket', async function () {
      const ros = await connect(wsUrl);
      try {
        const goal = await ros.action('/fibonacci', { order: 5 });
        await assertUtils.createDelay(20);
        assert.strictEqual(await goal.cancel(), undefined);
        const result = await goal.result;
        assert.deepStrictEqual(result, { sequence: [] });
        assert.strictEqual(goal.status, 'canceled');
        await assert.rejects(
          goal.cancel(),
          (error) => error.code === 'unknown_goal_id'
        );
      } finally {
        await ros.close();
      }
    });

    for (const { name, terminal, errorCode } of [
      { name: 'missing status', terminal: { payload: {} } },
      { name: 'unknown status', terminal: { status: 'unknown', payload: {} } },
      {
        name: 'invalid status',
        terminal: { status: 'executing', payload: {} },
      },
      {
        name: 'result error',
        terminal: { ok: false, code: 'action_failed', error: 'result failed' },
        errorCode: 'action_failed',
      },
      {
        name: 'connection loss before a result',
        terminal: null,
        errorCode: 'connection_lost',
      },
    ]) {
      it(`does not infer a terminal status from ${name}`, async function () {
        const wireServer = new WebSocketServer({ port: 0, host: '127.0.0.1' });
        wireServer.on('connection', (socket) => {
          socket.on('message', (data) => {
            const { id } = JSON.parse(data.toString());
            socket.send(
              JSON.stringify({ id, ok: true, payload: { accepted: true } })
            );
            if (terminal === null) {
              socket.close();
            } else {
              socket.send(
                JSON.stringify({ event: 'result', goalId: id, ...terminal })
              );
            }
          });
        });
        await once(wireServer, 'listening');
        let ros;
        try {
          ros = await connect(
            `ws://127.0.0.1:${wireServer.address().port}/capability`
          );
          const goal = await ros.action('/fibonacci', { order: 5 });
          if (errorCode) {
            await assert.rejects(
              goal.result,
              (error) => error.code === errorCode
            );
          } else {
            assert.deepStrictEqual(await goal.result, {});
          }
          assert.strictEqual(
            goal.status,
            terminal === null ? undefined : 'unknown'
          );
          assert.strictEqual(ros._ws._goals.size, 0);
        } finally {
          if (ros) await ros.close();
          await new Promise((resolve) => wireServer.close(resolve));
        }
      });
    }

    it('exposes an aborted status without changing the result payload', async function () {
      const capability = '/fibonacci_abort';
      let finishExecution;
      const execution = new Promise((resolve) => (finishExecution = resolve));
      const actionServer = new rclnodejs.ActionServer(
        node,
        fibonacci,
        capability,
        async (goalHandle) => {
          await execution;
          goalHandle.abort();
          return new Fibonacci.Result();
        }
      );
      runtime.expose({ action: { [capability]: fibonacci } });
      const ros = await connect(wsUrl);
      try {
        const goal = await ros.action(capability, { order: 5 });
        assert.strictEqual(goal.status, undefined);
        finishExecution();
        assert.deepStrictEqual(await goal.result, { sequence: [] });
        assert.strictEqual(goal.status, 'aborted');
        assert.throws(() => {
          goal.status = 'succeeded';
        }, TypeError);
      } finally {
        finishExecution();
        await ros.close();
        actionServer.destroy();
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
        assert.strictEqual(goal.status, 'succeeded');
        assert.throws(() => {
          goal.status = 'aborted';
        }, TypeError);
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

    it('accepts CRLF-framed HTTP action events', async function () {
      const crlfServer = http.createServer((req, res) => {
        res.writeHead(200, { 'content-type': 'text/event-stream' });
        res.end(
          'event: result\r\ndata: {"payload":{"sequence":[1,2,3]}}\r\n\r\n'
        );
      });
      await new Promise((resolve) =>
        crlfServer.listen(0, '127.0.0.1', resolve)
      );
      const address = crlfServer.address();
      const ros = await connect(`http://127.0.0.1:${address.port}`);
      try {
        const goal = await ros.action('/fibonacci', { order: 5 });
        assert.deepStrictEqual(await goal.result, { sequence: [1, 2, 3] });
        assert.strictEqual(goal.status, 'unknown');
      } finally {
        await ros.close();
        await new Promise((resolve, reject) =>
          crlfServer.close((err) => (err ? reject(err) : resolve()))
        );
      }
    });

    it('rejects the result when HTTP stream setup fails', async function () {
      const originalFetch = globalThis.fetch;
      globalThis.fetch = async () => ({
        ok: true,
        body: {
          getReader() {
            throw new Error('reader unavailable');
          },
        },
      });
      const ros = await connect('http://127.0.0.1:1');
      try {
        const goal = await ros.action('/fibonacci', { order: 5 });
        await assert.rejects(goal.result, (err) => {
          assert.strictEqual(err.code, 'network_error');
          assert.match(err.message, /reader unavailable/);
          return true;
        });
      } finally {
        globalThis.fetch = originalFetch;
        await ros.close();
      }
    });
  });
});
