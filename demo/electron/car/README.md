# ROS 2 Car Control Demo (Electron + rclnodejs)

Interactive car-control app: a virtual joystick publishes `geometry_msgs/Twist`
velocity commands on `/cmd_vel`, and a visualization subscribes and moves a car
in real time.

![demo screenshot](./car-control-electron.gif)

## Features

- Joystick plus WASD / arrow-key control with an emergency stop
- Real-time car movement, position tracking, and command counter
- ROS 2 node `car_control_node` publishing and subscribing on `cmd_vel`

## Prerequisites

- ROS 2 (Humble, Jazzy, Kilted, Lyrical, or Rolling), sourced
- Node.js >= 20.20.2
- Linux (prebuilt rclnodejs binaries are provided for Ubuntu)

## Install & Run

```bash
cd demo/electron/car
source /opt/ros/<distro>/setup.bash   # required before install and launch
npm install
npm start
```

rclnodejs ships prebuilt Electron binaries and selects the matching one at
runtime from `ROS_DISTRO` + Linux codename + architecture, so no compilation is
needed. Do not run `electron-rebuild` against rclnodejs — it rebuilds from
source and bypasses the prebuilt binary (the Forge `rebuildConfig` in
`package.json` already excludes `rclnodejs`).

## Controls

- Mouse: directional buttons (↑ ↓ ← →) and the red STOP button
- Keyboard: W/S forward/back, A/D turn, Space or Esc to stop

## Message Format & Mapping

`geometry_msgs/Twist` on `/cmd_vel`:

| Command  | linear.x | angular.z |
| -------- | -------- | --------- |
| Forward  | +1.0     | 0.0       |
| Backward | -1.0     | 0.0       |
| Left     | 0.0      | +1.0      |
| Right    | 0.0      | -1.0      |
| Stop     | 0.0      | 0.0       |

## Test from the CLI

```bash
ros2 topic echo /cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
```

## Packaging

```bash
npm run package   # standalone app in out/
npm run make      # zip / deb / rpm installers (needs zip, dpkg, fakeroot)
```

ASAR is enabled but `rclnodejs` is unpacked (`asar.unpack` in `package.json`)
because it needs filesystem access to its generated code and native bindings.
The target machine must still have ROS 2 installed and sourced — rclnodejs links
dynamically to ROS 2 shared libraries:

```bash
source /opt/ros/<distro>/setup.bash
./out/rclnodejs-electron-car-demo-linux-x64/rclnodejs-electron-car-demo
```

## Troubleshooting

- **"Failed to initialize ROS2"** — source ROS 2 before `npm start`.
- **Native module fails to load** — ensure `ROS_DISTRO` is set so the prebuilt
  binary matches; as a last resort set `RCLNODEJS_FORCE_BUILD=1` (needs a
  compiler toolchain and network access).
- **Topic missing** — check `ros2 daemon status` and `ros2 topic list`.
- **Car not moving** — open the DevTools console and check for errors.

## License

Apache License 2.0, same as rclnodejs.
