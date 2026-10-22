// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
// Licensed under the Apache License, Version 2.0.

import { once } from 'node:events';
import readline from 'node:readline';
import { JointController, MESSAGE_TYPE, STEP, TOPIC } from './joint-state.js';
import { createStaticServer, parsePort } from './static.mjs';

const HELP = `
Keyboard controls (focus this terminal, not the browser):
  A / D or Left / Right   Base: - / + 5 degrees (limits: -180 to 180)
  S / W or Down / Up      Elbow: - / + 5 degrees (limits: -135 to 135)
  Space                  Start / stop automatic motion
  R                      Reset both joints to zero
  H                      Show this help
  Q / Esc / Ctrl+C       Quit
Manual movement stops automatic motion. This is a virtual arm, not a hardware driver.
`;

let rclnodejs;
let runtime;
let server;
let publishTimer;
let rosInitialized = false;
let stopping;
let previousRawMode;
const state = new JointController();
const input = process.stdin;

function printPosition() {
  const angles = state.positions.map((value) =>
    ((value * 180) / Math.PI).toFixed(1)
  );
  const line = `Base ${angles[0]} deg | Elbow ${angles[1]} deg | ${state.animated ? 'automatic' : 'manual'}`;
  if (process.stdout.isTTY) {
    readline.clearLine(process.stdout, 0);
    readline.cursorTo(process.stdout, 0);
    process.stdout.write(line);
  } else {
    console.log(line);
  }
}

function stop(code = 0) {
  if (stopping) return stopping;
  process.exitCode = code;
  clearInterval(publishTimer);
  input.removeListener('keypress', onKey);
  input.pause();
  if (previousRawMode !== undefined) input.setRawMode(previousRawMode);
  input.destroy();
  stopping = (async () => {
    console.log('\nStopping manipulator demo...');
    const deadline = setTimeout(() => {
      console.error('Shutdown timed out');
      process.exit(1);
    }, 5000);
    try {
      if (runtime) await runtime.stop();
      if (server?.listening) {
        server.closeAllConnections();
        await new Promise((resolve, reject) => {
          server.close((error) => (error ? reject(error) : resolve()));
        });
      }
    } finally {
      try {
        if (rosInitialized) rclnodejs.shutdown();
      } finally {
        clearTimeout(deadline);
      }
    }
  })().catch((error) => {
    console.error('Unable to shut down the demo:', error);
    process.exitCode = 1;
  });
  return stopping;
}

function onKey(character, key = {}) {
  if (
    (key.ctrl && ['c', 'd'].includes(key.name)) ||
    ['q', 'escape'].includes(key.name)
  ) {
    void stop();
    return;
  }
  switch (key.name) {
    case 'a':
    case 'left':
      state.move(0, -STEP);
      break;
    case 'd':
    case 'right':
      state.move(0, STEP);
      break;
    case 's':
    case 'down':
      state.move(1, -STEP);
      break;
    case 'w':
    case 'up':
      state.move(1, STEP);
      break;
    case 'space':
      state.toggleAnimation();
      break;
    case 'r':
      state.reset();
      break;
    case 'h':
      console.log(HELP);
      break;
    default:
      if (character?.trim())
        console.log(
          `\nUnknown key ${JSON.stringify(character)}; press H for help.`
        );
      return;
  }
  printPosition();
}

async function main() {
  const host = process.env.HOST ?? '127.0.0.1';
  const staticPort = parsePort(
    process.env.STATIC_PORT ?? '8080',
    'STATIC_PORT'
  );
  const runtimePort = parsePort(
    process.env.RUNTIME_PORT ?? '9000',
    'RUNTIME_PORT'
  );
  if (!process.env.ROS_DISTRO || !process.env.AMENT_PREFIX_PATH) {
    throw new Error('Source your ROS 2 setup.bash before starting this demo');
  }

  // Source-checkout imports avoid rebuilding dist; downstream apps use
  // 'rclnodejs' and 'rclnodejs/web/server'.
  ({ default: rclnodejs } = await import('../../../index.js'));
  const { createRuntime, WebSocketTransport } =
    await import('../../../lib/runtime/index.js');
  rclnodejs.removeSignalHandlers();
  await rclnodejs.init();
  rosInitialized = true;
  const node = rclnodejs.createNode('web_manipulator_controller');
  const publisher = node.createPublisher(MESSAGE_TYPE, TOPIC);
  const transport = new WebSocketTransport({ host, port: runtimePort });
  runtime = createRuntime({ node, transport });
  runtime.expose({ subscribe: { [TOPIC]: MESSAGE_TYPE } });
  rclnodejs.spin(node);
  await runtime.start();
  server = createStaticServer(transport.port);
  server.listen(staticPort, host);
  await once(server, 'listening');
  server.on('error', (error) => {
    console.error('Static server failed:', error);
    void stop(1);
  });

  const displayHost = host === '0.0.0.0' || host === '::' ? 'localhost' : host;
  const urlHost = displayHost.includes(':') ? `[${displayHost}]` : displayHost;
  console.log(`Static page : http://${urlHost}:${server.address().port}/`);
  console.log(`WebSocket   : ws://${urlHost}:${transport.port}/capability`);
  console.log(
    `Publishing ${TOPIC} (${MESSAGE_TYPE}) at 10 Hz; browser access is read-only.`
  );
  console.log(HELP);
  if (!input.isTTY)
    console.log(
      'Non-interactive stdin: key characters can be piped in; EOF stops the demo.'
    );
  readline.emitKeypressEvents(input);
  if (input.isTTY) {
    previousRawMode = Boolean(input.isRaw);
    input.setRawMode(true);
  }
  input.on('keypress', onKey);
  input.once('end', () => void stop());
  input.once('error', (error) => {
    console.error('Terminal input failed:', error);
    void stop(1);
  });
  input.resume();
  process.once('SIGINT', () => void stop());
  process.once('SIGTERM', () => void stop());
  let previousTick = performance.now();
  publishTimer = setInterval(() => {
    try {
      const now = performance.now();
      state.tick((now - previousTick) / 1000);
      previousTick = now;
      publisher.publish(state.toMessage());
      if (process.stdout.isTTY) printPosition();
    } catch (error) {
      console.error('Unable to publish joint states:', error);
      void stop(1);
    }
  }, 100);
  printPosition();
}

main().catch(async (error) => {
  console.error('Manipulator demo failed:', error);
  await stop(1);
});
