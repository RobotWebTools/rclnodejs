# Electron Car Demo

The controls publish `geometry_msgs/msg/Twist` on `/cmd_vel`, and the simulated car moves when it receives those commands. Use a separate ROS graph if a real robot listens on `/cmd_vel`.

![Electron car demo](./car-control-electron.gif)

## Run

After [Electron setup](../README.md#setup), run from the repository root:

```bash
cd demo/electron/car
npm install
npm start
```

Drive with the on-screen arrows, WASD, or the arrow keys. Click **STOP** or press Space or Escape to stop.

In another terminal with ROS sourced, watch the velocity commands:

```bash
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist
```

For a standalone build, see [packaging](../README.md#package-a-demo).