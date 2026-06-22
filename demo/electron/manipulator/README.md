# Two-Joint Manipulator Demo (Electron + rclnodejs)

Interactive 3D visualization of a two-joint robotic arm using rclnodejs and
Three.js. It publishes and subscribes to `sensor_msgs/msg/JointState` on
`/joint_states`.

![demo screenshot](./manipulator-demo.gif)

## Features

- Real-time Three.js 3D arm with orbit controls
- Manual joint sliders and automatic sinusoidal animation
- Publishes `/joint_states` at 10 Hz; subscribes and visualizes incoming states
- Color-coded joints/markers with live position and message-rate display

## Prerequisites

- ROS 2 (Humble, Jazzy, Kilted, Lyrical, or Rolling), sourced
- Node.js >= 20.20.2
- Linux recommended (prebuilt rclnodejs binaries are provided for Ubuntu)

## Install & Run

```bash
cd demo/electron/manipulator
source /opt/ros/<distro>/setup.bash   # required before install and launch
npm install
npm start
```

rclnodejs ships prebuilt Electron binaries and selects the matching one at
runtime from `ROS_DISTRO` + Linux codename + architecture, so no compilation is
needed. Do not run `electron-rebuild` against rclnodejs — it rebuilds from
source and bypasses the prebuilt binary (the Forge `rebuildConfig` in
`package.json` already excludes `rclnodejs`). If no matching prebuilt binary
exists for your platform, rclnodejs falls back to building from source.

> ROS 2 must be sourced even for local-only visualization, so the prebuilt
> binary can be selected.

## Usage

- **Joint 1 (base)**: ±180° rotation; **Joint 2 (elbow)**: ±135° bend
- Sliders for manual control; "Start Animation" for automatic motion;
  "Reset Position" to return to zero
- 3D view: drag to orbit, mouse wheel to zoom

## ROS 2 Topics

`/joint_states` (`sensor_msgs/msg/JointState`) — joints `['joint1', 'joint2']`,
positions in radians, published at 10 Hz. Monitor with:

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

## Packaging

```bash
npm run package   # standalone app in out/rclnodejs-manipulator-demo-linux-x64/
npm run make      # zip installer
```

ASAR is enabled but `rclnodejs` is unpacked (`asar.unpack` in `package.json`)
because it needs filesystem access to its generated code and native bindings.
The target machine must still have ROS 2 installed and sourced:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./out/rclnodejs-manipulator-demo-linux-x64/rclnodejs-manipulator-demo
```

## Files

- `main.js` — Electron main process + ROS 2 node (joint publishing / animation)
- `renderer.js` — Three.js scene, markers, and UI controls
- `index.html` — UI layout

## Troubleshooting

- **"librcl.so not found"** — source ROS 2, then `npm start`.
- **Native module fails to load** — ensure `ROS_DISTRO` is set so the prebuilt
  binary matches; to force a from-source rebuild set `RCLNODEJS_FORCE_BUILD=1`.
- **No messages** — run `ros2 daemon start`, then check `ros2 topic list` and
  `ros2 topic echo /joint_states`.
- **Blank 3D scene** — open DevTools (F12) and check for Three.js / WebGL errors.

## License

Apache License 2.0, same as rclnodejs.
