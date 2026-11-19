# Electron Demos

- [Topics](topics/README.md): send and receive ROS 2 messages in a desktop window.
- [Car](car/README.md): drive a simulated car with buttons or the keyboard.
- [Manipulator](manipulator/README.md): move two joints and view the arm in Three.js.
- [Turtle TF2](turtle_tf2/README.md): view coordinate frames and make one turtle follow another.

The Electron main process runs rclnodejs. The window sends commands and receives ROS data over IPC.

## Setup

Use Linux with a desktop session, Node.js 20.20.2 or newer, and ROS 2. Source ROS before installing or starting a demo. Replace `lyrical` with your distro:

```bash
source /opt/ros/lyrical/setup.bash
```

At launch, rclnodejs uses a prebuilt Electron binary if one matches your ROS distro, Ubuntu version, and architecture. Forge skips rebuilding rclnodejs. If you need to compile it, see the [native build requirements](../../docs/BUILDING.md#requirements).

## Package a Demo

In the demo directory, `npm run package` puts the app in `out/`. `npm run make` builds the distributables. For Car, Topics, and Turtle TF2, install `dpkg` and `fakeroot` for `.deb` packages and `rpm` or `rpm-build` for `.rpm` packages.

Install and source ROS 2 on the target machine before starting the packaged app.

For ROS 2 basics, see [Writing Your First rclnodejs Nodes](../../tutorials/writing-your-first-rclnodejs-nodes.md).