// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Browser entry. Demonstrates the rclnodejs Web Runtime browser SDK
// with **zero glue** — no hand-written message shapes, no shared
// types module. Every type comes from rclnodejs's auto-generated
// MessagesMap / ServicesMap, looked up by the ROS interface name
// passed as a single string generic at the call site.

import type {} from 'rclnodejs';
import {
  connect,
  RosClient,
  type ActionHandle,
  type ActionResult,
  type Subscription,
} from 'rclnodejs/web';
import './style.css';

type Mode = 'ws' | 'http';

interface ActionRun {
  client: RosClient;
  mode: Mode;
  goal?: ActionHandle<ActionResult<'example_interfaces/action/Fibonacci'>>;
  cancelPending: boolean;
}

const HOST = location.hostname || 'localhost';
const parameters = new URLSearchParams(location.search);
const ENDPOINTS: Record<Mode, string> = {
  ws: `ws://${HOST}:${parameters.get('wsPort') || 9000}/capability`,
  // HTTP base for call/publish.
  http: `http://${HOST}:${parameters.get('httpPort') || 9001}`,
};
// Pass Form C ({http, ws}) when the user picks HTTP so subscribe still
// reaches the WS runtime on :9000. The SDK's auto-derived sibling would
// land on :9001 instead and fail — this dev layout splits the two
// transports across separate ports.
function connectArg(mode: Mode): string | { http: string; ws: string } {
  return mode === 'http'
    ? { http: ENDPOINTS.http, ws: ENDPOINTS.ws }
    : ENDPOINTS.ws;
}

function $<T extends HTMLElement>(id: string): T {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing element #${id}`);
  return el as T;
}

function setStatus(text: string, cls: 'ok' | 'err' | '' = ''): void {
  const el = $('status');
  el.textContent = text;
  el.className = `status ${cls}`;
}

function setEndpoint(mode: Mode): void {
  $('endpoint').textContent =
    mode === 'http'
      ? `${ENDPOINTS.http}  (subscribe routed to ${ENDPOINTS.ws})`
      : ENDPOINTS.ws;
}

function log(id: string, text: string, cls: 'ok' | 'err' | '' = ''): void {
  const el = $(id);
  const line = document.createElement('div');
  line.textContent = `${new Date().toLocaleTimeString()}  ${text}`;
  if (cls) line.className = cls;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}

async function main(): Promise<void> {
  let ros: RosClient | undefined;
  let tickSub: Subscription | undefined;
  let mode: Mode = 'ws';
  let connectionVersion = 0;
  let activeAction: ActionRun | undefined;
  const actionForm = $<HTMLFormElement>('actionForm');
  const actionOrder = $<HTMLInputElement>('actionOrder');
  const actionSendBtn = $<HTMLButtonElement>('actionSendBtn');
  const actionCancelBtn = $<HTMLButtonElement>('actionCancelBtn');
  const actionStopBtn = $<HTMLButtonElement>('actionStopBtn');
  const actionResult = $('actionResult');

  function setActionStatus(text: string, cls: 'ok' | 'err' | '' = ''): void {
    const output = $('actionStatus');
    output.textContent = text;
    output.className = `status ${cls}`;
  }

  function updateActionControls(): void {
    actionSendBtn.disabled = !ros || !!activeAction;
    actionOrder.disabled = !!activeAction;
    actionCancelBtn.disabled =
      !activeAction?.goal ||
      activeAction.mode !== 'ws' ||
      activeAction.cancelPending;
    actionCancelBtn.title =
      mode === 'http'
        ? 'HTTP actions cannot be canceled; use WebSocket'
        : 'Request cancellation over WebSocket';
    actionStopBtn.disabled = !activeAction || activeAction.mode !== 'http';
    actionForm.setAttribute('aria-busy', String(!!activeAction));
  }

  async function detachAction(): Promise<void> {
    const current = activeAction;
    if (!current) return;
    activeAction = undefined;
    setActionStatus('Detached');
    log('actionLog', 'Disconnected locally; the ROS goal may continue.');
    updateActionControls();
    await current.client.close();
  }

  actionForm.onsubmit = async (event): Promise<void> => {
    event.preventDefault();
    if (!ros || activeAction || !actionForm.reportValidity()) return;
    const order = actionOrder.valueAsNumber;
    if (!Number.isInteger(order) || order < 2 || order > 12) return;
    const client = new RosClient(
      mode === 'http' ? { http: ENDPOINTS.http } : ENDPOINTS.ws
    );
    const current: ActionRun = { client, mode, cancelPending: false };
    activeAction = current;
    $('actionLog').textContent = '';
    actionResult.textContent = '-';
    setActionStatus('Submitting');
    updateActionControls();
    try {
      await client.connect();
      if (activeAction !== current) return;
      const goal = await client.action<'example_interfaces/action/Fibonacci'>(
        '/fibonacci',
        { order },
        {
          onFeedback(feedback) {
            if (activeAction !== current) return;
            log('actionLog', `Feedback: ${JSON.stringify(feedback.sequence)}`);
          },
        }
      );
      if (activeAction !== current) {
        void goal.result.catch(() => {});
        return;
      }
      current.goal = goal;
      setActionStatus('Running');
      updateActionControls();
      const result = await goal.result;
      if (activeAction !== current) return;
      actionResult.textContent = JSON.stringify(result.sequence);
      setActionStatus(
        goal.status || 'unknown',
        goal.status === 'succeeded' ? 'ok' : ''
      );
      log(
        'actionLog',
        `Result: ${goal.status}`,
        goal.status === 'succeeded' ? 'ok' : ''
      );
    } catch (error) {
      if (activeAction === current) {
        const failure = error as { code?: string; message?: string };
        setActionStatus('Failed', 'err');
        log(
          'actionLog',
          `${failure.code || 'action_failed'}: ${failure.message}`,
          'err'
        );
      }
    } finally {
      if (activeAction === current) {
        activeAction = undefined;
        updateActionControls();
      }
      await client.close();
    }
  };

  actionCancelBtn.onclick = async (): Promise<void> => {
    const current = activeAction;
    if (!current?.goal || current.mode !== 'ws' || current.cancelPending)
      return;
    current.cancelPending = true;
    setActionStatus('Canceling');
    updateActionControls();
    try {
      await current.goal.cancel();
    } catch (error) {
      if (activeAction !== current) return;
      current.cancelPending = false;
      setActionStatus('Running');
      log('actionLog', `Cancel failed: ${String(error)}`, 'err');
      updateActionControls();
    }
  };
  actionStopBtn.onclick = () => detachAction();

  async function teardown(): Promise<void> {
    const previousSub = tickSub;
    const previousClient = ros;
    tickSub = undefined;
    ros = undefined;
    updateActionControls();
    await detachAction();
    if (previousSub) {
      try {
        await previousSub.close();
      } catch {
        /* noop */
      }
      $<HTMLButtonElement>('subBtn').disabled = false;
      $<HTMLButtonElement>('unsubBtn').disabled = true;
    }
    if (previousClient) {
      try {
        await previousClient.close();
      } catch {
        /* noop */
      }
    }
  }

  async function reconnect(nextMode: Mode): Promise<void> {
    const version = ++connectionVersion;
    mode = nextMode;
    await teardown();
    if (version !== connectionVersion) return;
    setEndpoint(mode);
    setStatus(`connecting (${mode})…`);
    let connected: RosClient;
    try {
      connected = await connect(connectArg(mode));
      if (version !== connectionVersion) {
        await connected.close();
        return;
      }
      ros = connected;
      setStatus(`connected (${mode})`, 'ok');
      updateActionControls();
    } catch (e) {
      if (version !== connectionVersion) return;
      setStatus(`failed: ${String(e)}`, 'err');
      return;
    }

    // Always-on chatter subscription; subscribe always uses WS — the
    // explicit { ws } in connectArg() makes this work in HTTP mode too.
    try {
      await connected.subscribe<'std_msgs/msg/String'>(
        '/web_demo_chatter',
        (msg) =>
          version === connectionVersion && log('chatLog', `<- ${msg.data}`)
      );
    } catch (e) {
      if (version !== connectionVersion) return;
      const err = e as { message?: string; code?: string };
      log('chatLog', `subscribe failed: ${err.message} (${err.code})`, 'err');
    }
  }

  // Wire button handlers once. They use the live `ros` ref so they
  // work across reconnect()s.

  // 1. Service call.
  $<HTMLButtonElement>('callBtn').onclick = async (): Promise<void> => {
    if (!ros) return;
    const a = Number($<HTMLInputElement>('addA').value);
    const b = Number($<HTMLInputElement>('addB').value);
    try {
      const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
        '/add_two_ints',
        { a: `${a}n`, b: `${b}n` }
      );
      log('callLog', `${a} + ${b} = ${reply.sum}`, 'ok');
    } catch (e) {
      const err = e as { message?: string; code?: string };
      log('callLog', `error: ${err.message} (${err.code})`, 'err');
    }
  };

  // 2. Subscription.
  const subBtn = $<HTMLButtonElement>('subBtn');
  const unsubBtn = $<HTMLButtonElement>('unsubBtn');
  subBtn.onclick = async (): Promise<void> => {
    if (!ros) return;
    try {
      tickSub = await ros.subscribe<'std_msgs/msg/String'>(
        '/web_demo_tick',
        (msg) => log('tickLog', msg.data)
      );
      subBtn.disabled = true;
      unsubBtn.disabled = false;
      log('tickLog', `subscribed (subId=${tickSub.subId})`, 'ok');
    } catch (e) {
      const err = e as { message?: string; code?: string };
      log('tickLog', `error: ${err.message} (${err.code})`, 'err');
    }
  };
  unsubBtn.onclick = async (): Promise<void> => {
    if (!tickSub) return;
    await tickSub.close();
    tickSub = undefined;
    unsubBtn.disabled = true;
    subBtn.disabled = false;
    log('tickLog', 'unsubscribed', 'ok');
  };

  // 3. Topic publish (chatter is subscribed in reconnect()).
  $<HTMLButtonElement>('pubBtn').onclick = async (): Promise<void> => {
    if (!ros) return;
    const data = $<HTMLInputElement>('chatMsg').value;
    try {
      await ros.publish<'std_msgs/msg/String'>('/web_demo_chatter', { data });
      log('chatLog', `-> ${data}`, 'ok');
    } catch (e) {
      const err = e as { message?: string; code?: string };
      log('chatLog', `error: ${err.message} (${err.code})`, 'err');
    }
  };

  // 4. Allow-list rejection (untyped fallback overload).
  $<HTMLButtonElement>('badCallBtn').onclick = async (): Promise<void> => {
    if (!ros) return;
    try {
      await ros.call('/dangerous', {});
      log(
        'badLog',
        'unexpected success — registry should have rejected',
        'err'
      );
    } catch (e) {
      const err = e as { message?: string; code?: string };
      log('badLog', `rejected: ${err.message} (${err.code})`, 'ok');
    }
  };

  // Transport toggle.
  for (const radio of document.querySelectorAll<HTMLInputElement>(
    'input[name="transport"]'
  )) {
    radio.addEventListener('change', (e) =>
      reconnect((e.target as HTMLInputElement).value as Mode)
    );
  }

  window.addEventListener('pagehide', () => {
    connectionVersion++;
    void teardown();
  });

  await reconnect('ws');
}

main().catch((err: unknown) => {
  console.error(err);
  setStatus(`fatal: ${String(err)}`, 'err');
});
