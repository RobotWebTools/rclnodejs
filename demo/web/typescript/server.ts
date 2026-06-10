// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// TypeScript demo server. Run with `npm run server` (which uses tsx) or
// `npx tsx server.ts`. Behaviour matches demo/web/javascript/runtime.cjs
// — same runtime + same demo nodes — except this side is written in
// TypeScript so the typed SDK story is visible end to end. The static
// page server is Vite (`npm run dev`), parallel to the JS demo's
// separate `node static.cjs`.

// rclnodejs is a CommonJS module without first-class ESM types; using
// require keeps the server independent of how a downstream project
// configures TypeScript module resolution.
// eslint-disable-next-line @typescript-eslint/no-var-requires
import { createRequire } from 'node:module';
const require_ = createRequire(import.meta.url);

// eslint-disable-next-line @typescript-eslint/no-explicit-any
const rclnodejs: any = require_('rclnodejs');
const { createRuntime, WebSocketTransport, HttpTransport } = require_(
  'rclnodejs/web/server'
);

const RUNTIME_PORT = Number(process.env.RUNTIME_PORT || 9000);
const HTTP_PORT = Number(process.env.HTTP_PORT || 9001);

// Render the registry as a small human-readable table — see the matching
// helper in demo/web/javascript/runtime.cjs.
function formatCapabilities(
  caps: Record<'call' | 'publish' | 'subscribe', Record<string, string>>
): string {
  const rows: Array<[string, string, string]> = [];
  for (const verb of ['call', 'publish', 'subscribe'] as const) {
    for (const [topic, type] of Object.entries(caps[verb] || {})) {
      rows.push([verb, topic, type]);
    }
  }
  if (rows.length === 0) return '  (none)';
  const w0 = Math.max(...rows.map((r) => r[0].length));
  const w1 = Math.max(...rows.map((r) => r[1].length));
  return rows
    .map(([v, t, ty]) => `  ${v.padEnd(w0)}  ${t.padEnd(w1)}  ${ty}`)
    .join('\n');
}

async function main(): Promise<void> {
  await rclnodejs.init();
  const node = rclnodejs.createNode('rclnodejs_web_ts_demo_node');

  // Service the browser will call.
  node.createService(
    'example_interfaces/srv/AddTwoInts',
    '/add_two_ints',
    (
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      request: any,
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      response: any
    ): void => {
      const reply = response.template;
      reply.sum = request.a + request.b;
      response.send(reply);
    }
  );

  // 1 Hz tick publisher so the browser's subscribe() shows live data
  // without the user having to publish first.
  const tickPub = node.createPublisher('std_msgs/msg/String', '/web_demo_tick');
  let counter = 0;
  setInterval(() => {
    tickPub.publish({
      data: `tick ${counter++} @ ${new Date().toISOString()}`,
    });
  }, 1000);

  rclnodejs.spin(node);

  const runtime = createRuntime({
    node,
    transports: [
      new WebSocketTransport({
        port: RUNTIME_PORT,
        // Dual-stack — see the matching note in the JS demo.
        host: '::',
      }),
      // HTTP for `call` / `publish` (curl, Postman, AI agents).
      // Same registry / dispatcher — the L2 seam in action.
      new HttpTransport({
        port: HTTP_PORT,
        host: '::',
      }),
    ],
  });

  runtime.expose({
    call: { '/add_two_ints': 'example_interfaces/srv/AddTwoInts' },
    publish: { '/web_demo_chatter': 'std_msgs/msg/String' },
    subscribe: {
      '/web_demo_tick': 'std_msgs/msg/String',
      '/web_demo_chatter': 'std_msgs/msg/String',
    },
  });
  await runtime.start();

  const caps = runtime.registry.list();
  const total =
    Object.keys(caps.call || {}).length +
    Object.keys(caps.publish || {}).length +
    Object.keys(caps.subscribe || {}).length;

  console.log('rclnodejs/web demo running (TypeScript)');
  console.log(`  WebSocket : ws://localhost:${RUNTIME_PORT}/capability`);
  console.log(
    `  HTTP      : http://localhost:${HTTP_PORT}/capability  (call / publish, curl-able)`
  );
  console.log();
  console.log(`Exposed capabilities (${total}):`);
  console.log(formatCapabilities(caps));
  console.log();
  console.log(
    'Static page: run `npm run dev` in another shell, then open http://localhost:8080/'
  );

  const stop = async (): Promise<void> => {
    console.log('\nstopping…');
    await runtime.stop();
    rclnodejs.shutdown();
    process.exit(0);
  };
  process.once('SIGINT', stop);
  process.once('SIGTERM', stop);
}

main().catch((err: unknown) => {
  console.error(err);
  process.exit(1);
});
