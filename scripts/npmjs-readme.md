# rclnodejs

`rclnodejs` is a Node.js client library for ROS 2 that provides JavaScript and TypeScript APIs for building ROS 2 applications.

Supported ROS 2 distributions include Humble, Jazzy, Kilted, Lyrical, and Rolling.

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = new rclnodejs.Node('publisher_example_node');
const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
publisher.publish(`Hello ROS 2 from rclnodejs`);
node.spin();
```

This example assumes your ROS 2 environment is already sourced.

## Installation

### Prerequisites

- [Node.js](https://nodejs.org/en/) version >= 20.20.2
- [ROS 2 SDK](https://docs.ros.org/en/lyrical/Installation.html)

Before installing or running rclnodejs, source your ROS 2 environment:

```bash
source /opt/ros/<distro>/setup.bash
```

### Install rclnodejs

```bash
npm i rclnodejs
```

To install from GitHub instead of npm, run:

```bash
npm install RobotWebTools/rclnodejs#<branch>
```

### Prebuilt Binaries

rclnodejs ships with prebuilt native binaries for common Linux configurations, so most installs skip compilation.

**Supported Platforms:**

- **Ubuntu 22.04 (Jammy)** - ROS 2 Humble
- **Ubuntu 24.04 (Noble)** - ROS 2 Jazzy, Kilted
- **Ubuntu 26.04 (Resolute)** - ROS 2 Lyrical
- **Architectures:** x64, arm64
- **Node.js:** >= 20.20.2 (N-API compatible)

Installations outside this matrix automatically fall back to building from source. To force a source build even when a prebuilt binary is available:

```bash
export RCLNODEJS_FORCE_BUILD=1
npm install rclnodejs
```

## Documentation and Examples

- API documentation: [robotwebtools.github.io/rclnodejs/docs](https://robotwebtools.github.io/rclnodejs/docs/index.html)
- Tutorials: [tutorials/](https://github.com/RobotWebTools/rclnodejs/tree/develop/tutorials)
- JavaScript examples: [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example)
- TypeScript demos: [demo/typescript/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/typescript)
- Browser demos: [demo/web/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/web) (typed Web SDK) and [demo/rosocket/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/rosocket) (WebSocket gateway)
- Electron demos: [demo/electron/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/electron)
- Companion CLI: [rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs-cli/)

## Message Generation

rclnodejs auto-generates JavaScript message interfaces and TypeScript declarations during `npm install`, so in most projects you do not need to run anything by hand. If you install additional ROS packages **after** rclnodejs was installed, re-run the generator from your project so the new interfaces are picked up:

```bash
npx generate-ros-messages
```

Generated files are written to `<your-project>/node_modules/rclnodejs/generated/`.

## Using rclnodejs with TypeScript

TypeScript declaration files are included in the package. In most projects, configuring your `tsconfig.json` is sufficient:

```jsonc
{
  "compilerOptions": {
    "module": "NodeNext",
    "moduleResolution": "NodeNext",
    "target": "es2022",
  },
}
```

Then `import * as rclnodejs from 'rclnodejs'` works the same as the JavaScript example at the top of this README.

## ROS 2 in the browser

`rclnodejs` ships **two** ways to reach ROS 2 from the browser — pick one based on how much glue you want to write.

- **`rclnodejs/web`** — typed, allow-listed, `curl`-able browser SDK. A `web.json` file is your public API; the browser SDK types `call` / `publish` / `subscribe` end-to-end from your ROS 2 message types; every capability is also a plain HTTP endpoint (`curl -X POST http://<host>/capability/call/<name>`), with `subscribe` streaming as Server-Sent Events (`GET http://<host>/capability/subscribe/<name>`), so shell scripts, Postman, AI-agent tool-use, and a bare browser `fetch()` / `EventSource` (CORS-enabled) just work.

  ```ts
  import { connect } from 'rclnodejs/web';
  const ros = await connect('ws://host:9000/capability');
  const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
    '/add_two_ints', { a: '2n', b: '40n' }
  ); // reply.sum is typed as `${number}n`
  ```

  No SDK needed for subscribe — with the HTTP/SSE transport enabled (`--http-sse`, plus `--http-cors` for cross-origin), any browser streams a live ROS 2 topic via built-in `EventSource`:

  ```js
  const es = new EventSource('http://host:9001/capability/subscribe/chatter');
  es.onmessage = (e) => console.log(JSON.parse(e.data)); // live ROS 2 messages
  ```

  See the [Web SDK guide](https://github.com/RobotWebTools/rclnodejs/tree/develop/web).

- **`rosocket`** — thin WebSocket gateway, zero browser dependencies (just built-in `WebSocket` + `JSON`). Best for quick prototypes and `roslibjs`-style apps.

  ```bash
  npx rosocket --port 9000 --topic /chatter:std_msgs/msg/String
  ```

  See the [rosocket guide](https://github.com/RobotWebTools/rclnodejs/tree/develop/rosocket).

## License

Apache License 2.0
