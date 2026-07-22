# rclnodejs v2.1.0

First stable release in the 2.1 line. The headline is a full ES Module migration: rclnodejs is now a native ESM package that ships both ESM and CommonJS from a single artifact, so `import` and `require` consumers — and TypeScript under `NodeNext` — all resolve the right entry automatically.

This release promotes the 2.1.0-beta.0 baseline to stable and rolls in the fixes and maintenance that landed during the beta cycle. Upgrading from 2.0.0 is seamless — `require('rclnodejs')` keeps working unchanged, and you can now also `import` it without a shim or interop wrapper.

The migration landed in three staged phases (rename CJS-only tooling → convert the library and tests to native ESM → add the dual build), so the git history stays bisectable and each step is reviewable on its own.

> 🎉 **A milestone release** — 2.1.0 is the **100th release of rclnodejs** on [npm](https://www.npmjs.com/package/rclnodejs?activeTab=versions). Thank you to everyone who has contributed code, issues, and feedback over the years to get us here. 🙌

## 📣 Important Notes

- rclnodejs is now `"type": "module"`. The package publishes a tsup-built `dist/` that emits both an ESM (`import`) and a CommonJS (`require`) entry for every public surface, selected at resolution time through `package.json` `exports` conditions — covering the root entry plus the `rclnodejs/web`, `rclnodejs/web/server`, and `rclnodejs/rosocket` subpaths.
- CommonJS consumers are unaffected. `const rclnodejs = require('rclnodejs')` continues to work exactly as before, including the `rclnodejs/web` and `rclnodejs/rosocket` subpaths under the `require` condition.
- ESM consumers can now `import rclnodejs from 'rclnodejs'` and use top-level `await` directly, with no `createRequire` shim.
- TypeScript projects on `module`/`moduleResolution: NodeNext` get correct types for both module systems.
- Build and generator tooling (e.g. the message generator and JSDoc tooling) remain CommonJS and are kept on `.cjs` extensions; this is internal and does not affect consumers.
- Downstream code that reached into undocumented `lib/` internals via deep relative paths should switch to the published `exports`/subpath entry points.
- Minimum Node.js remains 20.20.2. The same Linux x64 / arm64 N-API prebuilds run unchanged on every Node.js ≥ 20.20.2 (including 24.x and 26.x) across Humble, Jazzy, Kilted, Lyrical, and Rolling.

## ✨ Highlights

### ESM migration — dual ESM + CommonJS package

- Phase 1 — rename CJS-only build and generator files to `.cjs`, isolating tooling that must stay CommonJS from the soon-to-be-ESM runtime ([#1529](https://github.com/RobotWebTools/rclnodejs/pull/1529))
- Phase 2 — convert `lib/`, `index.js`, and the full test suite to native ES modules ([#1530](https://github.com/RobotWebTools/rclnodejs/pull/1530))
- Phase 3 — dual ESM + CommonJS build via tsup, wired through `package.json` `exports` conditions so each consumer resolves the right artifact ([#1531](https://github.com/RobotWebTools/rclnodejs/pull/1531))
- Add type declarations for the `rclnodejs/rosocket` subpath — ships `.d.ts`/`.d.cts` so the WebSocket gateway types cleanly under both module systems ([#1533](https://github.com/RobotWebTools/rclnodejs/pull/1533))
- Modernize `tsconfig.json` for Node.js module resolution ([#1528](https://github.com/RobotWebTools/rclnodejs/pull/1528))

### API and types

- Add `enable_logger_service` option for runtime log-level control ([#1522](https://github.com/RobotWebTools/rclnodejs/pull/1522))
- Add `publisher`/`subscription` `event_type_is_supported` checks ([#1520](https://github.com/RobotWebTools/rclnodejs/pull/1520))
- Infer concrete types from constructor `TypeClass` forms for sharper type inference ([#1526](https://github.com/RobotWebTools/rclnodejs/pull/1526))

## 🛡️ Bug Fixes and Safeguards

- Fix native build failure on the Visual Studio 2026 toolset (Windows) ([#1539](https://github.com/RobotWebTools/rclnodejs/pull/1539))
- Electron demos load prebuilt binaries at runtime ([#1546](https://github.com/RobotWebTools/rclnodejs/pull/1546))

## 🔧 Maintenance

- Pump `commander` to 15.0.0 ([#1524](https://github.com/RobotWebTools/rclnodejs/pull/1524))
- Pump `node-gyp` to 13.0.0 ([#1544](https://github.com/RobotWebTools/rclnodejs/pull/1544))
- Pump `@types/node` ([#1549](https://github.com/RobotWebTools/rclnodejs/pull/1549))
- Modernize the TypeScript demos to NodeNext + ES2022 ([#1534](https://github.com/RobotWebTools/rclnodejs/pull/1534))
- Switch the npmjs README examples to ESM ([#1542](https://github.com/RobotWebTools/rclnodejs/pull/1542))

## 🚀 CI, Packaging, and Tooling

- Add a per-release GitHub release-notes link to the generated JSDoc sidebar ([#1551](https://github.com/RobotWebTools/rclnodejs/pull/1551))

## Getting Started with ESM

```js
// ESM
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = new rclnodejs.Node('publisher_example_node');
const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
publisher.publish('Hello ROS 2 from rclnodejs');
node.spin();
```

```js
// CommonJS — still supported, unchanged
const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish('Hello ROS 2 from rclnodejs');
  node.spin();
});
```

The `rclnodejs/web` and `rclnodejs/rosocket` subpaths resolve under both module systems:

```js
import { connect } from 'rclnodejs/web'; // ESM
const { connect } = require('rclnodejs/web'); // CommonJS
```

```bash
npm i rclnodejs
```

Full Changelog: [2.0.0...2.1.0](https://github.com/RobotWebTools/rclnodejs/compare/2.0.0...2.1.0)
