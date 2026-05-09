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

import { connect, type RosClient, type Subscription } from 'rclnodejs/web';
import './style.css';

type Mode = 'ws' | 'http';

const HOST = location.hostname || 'localhost';
const ENDPOINTS: Record<Mode, string> = {
  ws: `ws://${HOST}:9000/capability`,
  // HTTP base URL — the SDK derives the WS sibling automatically for
  // subscribe (see web/client.js _resolveUrls).
  http: `http://${HOST}:9001`,
};

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
      ? `${ENDPOINTS.http}  (subscribe lazily uses ws://${HOST}:9000/capability)`
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

  async function teardown(): Promise<void> {
    if (tickSub) {
      try {
        await tickSub.close();
      } catch {
        /* noop */
      }
      tickSub = undefined;
      $<HTMLButtonElement>('subBtn').disabled = false;
      $<HTMLButtonElement>('unsubBtn').disabled = true;
    }
    if (ros) {
      try {
        await ros.close();
      } catch {
        /* noop */
      }
      ros = undefined;
    }
  }

  async function reconnect(mode: Mode): Promise<void> {
    await teardown();
    setEndpoint(mode);
    setStatus(`connecting (${mode})…`);
    try {
      ros = await connect(ENDPOINTS[mode]);
      setStatus(`connected (${mode})`, 'ok');
    } catch (e) {
      setStatus(`failed: ${String(e)}`, 'err');
      return;
    }

    // Always-on chatter subscription; over HTTP this lazily opens
    // the WS sibling endpoint (subscribe always uses WS).
    try {
      await ros.subscribe<'std_msgs/msg/String'>('/web_demo_chatter', (msg) =>
        log('chatLog', `<- ${msg.data}`)
      );
    } catch (e) {
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

  await reconnect('ws');
}

main().catch((err: unknown) => {
  console.error(err);
  setStatus(`fatal: ${String(err)}`, 'err');
});
