# Turtle TF2 Demo (Electron + rclnodejs)

A rclnodejs port of ROS 2's `turtle_tf2_py`, with a Three.js 3D view of
coordinate-frame transforms. It broadcasts and listens to TF2 transforms, drives
turtlesim, and makes turtle2 follow turtle1.

![demo screenshot](./turtle-tf2-demo.gif)

## Features

- TF2 broadcasters: static, dynamic (circular motion), fixed-offset, and
  turtle-pose
- TF2 listener that drives turtle2 to follow turtle1
- 3D Three.js scene with orbit controls, colored frame axes, and a live
  transform list
- Controls to spawn turtle2, drive turtle1 (WASD), and toggle frame visibility

## Prerequisites

- ROS 2 (Humble, Jazzy, Kilted, Lyrical, or Rolling), sourced
- Node.js >= 20.20.2
- turtlesim: `sudo apt install ros-$ROS_DISTRO-turtlesim`
- Linux (prebuilt rclnodejs binaries are provided for Ubuntu)

## Install & Run

```bash
cd demo/electron/turtle_tf2
npm install
source /opt/ros/$ROS_DISTRO/setup.bash   # required, in the same terminal as npm start
ros2 run turtlesim turtlesim_node        # in another sourced terminal
npm start
```

rclnodejs ships prebuilt Electron binaries and selects the matching one at
runtime from `ROS_DISTRO` + Linux codename + architecture, so no compilation is
needed. Do not run `electron-rebuild` against rclnodejs — it rebuilds from
source and bypasses the prebuilt binary (the Forge `rebuildConfig` in
`package.json` already excludes `rclnodejs`).

In the app: click "Start Demo" to launch the broadcasters, "Spawn Turtle2" to
add the follower, then drive turtle1 with WASD and watch turtle2 follow.

## Coordinate Frames

```
world → carrot1_static → carrot1_dynamic   (dynamic orbits static, 2-unit radius)
world → turtle1 → carrot1_fixed
world → turtle2
```

Axes are colored X=red, Y=green, Z=blue, and the transform list shows live
positions and rotations.

## Controls

- **Turtle1**: W/S forward/back, A/D turn (click the 3D view first for keyboard
  focus)
- **Camera**: mouse drag to orbit, wheel to zoom, right-drag to pan
- **Frames**: toggle static (red), dynamic (orange), and fixed (purple) markers

turtle2 follows turtle1 proportionally (max ~2.0 units/s), correcting heading
and stopping within 0.5 units.

## Packaging

```bash
npm run package   # standalone app in out/
npm run make      # zip / deb / rpm installers (needs zip, dpkg, fakeroot)
```

The target machine must still have ROS 2 installed and sourced — rclnodejs links
dynamically to ROS 2 shared libraries:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./out/rclnodejs-turtle-tf2-demo-linux-x64/rclnodejs-turtle-tf2-demo
```

## Troubleshooting

- **"librcl.so: cannot open shared object file"** — source ROS 2 in the same
  terminal as `npm start`.
- **Can't spawn turtle2** — ensure `turtlesim_node` is running and
  `ros2 service list | grep spawn` shows the service.
- **No transforms** — click "Start Demo"; inspect with
  `ros2 run tf2_tools view_frames` or `ros2 topic echo /tf`.
- **Stuck on "Initializing ROS2 TF2 Demo..." or blank 3D view** — a WebGL
  failure. The app enables software WebGL (SwiftShader) for GPU-less
  environments like WSL2; if it still fails, the loading screen shows the error.
  Make sure your environment can create a WebGL context.
- **Native module errors** — ensure `ROS_DISTRO` is set so the prebuilt binary
  matches; to force a from-source rebuild set `RCLNODEJS_FORCE_BUILD=1`.

## License

Apache License 2.0, same as rclnodejs.
