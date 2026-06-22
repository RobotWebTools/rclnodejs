# Topics Demo (Electron + rclnodejs)

A minimal Electron app for ROS 2 topic communication: publish and subscribe to
`std_msgs/String` messages on `ts_demo`. A good starting point for rclnodejs
with Electron.

![demo screenshot](./electron-demo.gif)

## Features

- Text input to publish string messages on `ts_demo`
- Live display of received messages
- Published / received message counters
- ROS 2 node `electron_demo_node`

## Prerequisites

- ROS 2 (Humble, Jazzy, Kilted, Lyrical, or Rolling), sourced
- Node.js >= 20.20.2
- Linux (prebuilt rclnodejs binaries are provided for Ubuntu)

## Install & Run

```bash
cd demo/electron/topics
source /opt/ros/<distro>/setup.bash   # required before launch
npm install
npm start
```

rclnodejs ships prebuilt Electron binaries and picks the matching one at runtime
from `ROS_DISTRO` + Linux codename + architecture, so no compilation is needed.
Do not run `electron-rebuild` against rclnodejs — it rebuilds from source and
bypasses the prebuilt binary (the Forge `rebuildConfig` in `package.json`
already excludes `rclnodejs`).

The window provides a text field, a Send button (publishes to `ts_demo`), a
message display, and published / received counters.

## Packaging

```bash
npm run package   # standalone app in out/
npm run make      # zip / deb / rpm installers (needs zip, dpkg, fakeroot)
```

ASAR is enabled but `rclnodejs` is unpacked (`asar.unpack` in `package.json`)
because it needs filesystem access to its generated code and native bindings.
The target machine must still have ROS 2 installed and sourced:

```bash
source /opt/ros/<distro>/setup.bash
./out/rclnodejs-electron-demo-linux-x64/rclnodejs-electron-demo
```

## Files

- `main.js` — Electron main process + rclnodejs integration
- `renderer.js` — UI and ROS 2 communication
- `index.html` — interface layout

## License

Apache License 2.0, same as rclnodejs.
