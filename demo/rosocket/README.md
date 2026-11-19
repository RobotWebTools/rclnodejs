# Rosocket Demo

The browser uses [Rosocket](../../rosocket/README.md) to send and receive messages on `/chatter` and call `/add_two_ints`. No frontend build is needed.

## Run

Use Node.js 20.20.2 or newer and ROS 2 with `std_msgs` and `example_interfaces` installed. From the repository root, run these commands. Replace `lyrical` with your distro:

```bash
source /opt/ros/lyrical/setup.bash
npm install
HOST=127.0.0.1 node demo/rosocket/server.mjs
```

In a second terminal at the repository root, source the same distro and start the service:

```bash
source /opt/ros/lyrical/setup.bash
node example/services/service/service-example.mjs
```

Open your local `demo/rosocket/index.html` in a browser. Set the bridge URL to `ws://127.0.0.1:9000`. Click **Subscribe**, then **Publish**. Click **Call** to see a service reply.

## Check from ROS 2

With the browser subscribed, publish from another ROS-sourced terminal:

```bash
ros2 topic pub /chatter std_msgs/msg/String '{data: "hi from ROS 2"}' --once
```

Look for the message in the browser log. Use `ros2 topic echo /chatter std_msgs/msg/String` to see messages sent from the page. Stop the bridge and service with Ctrl+C.

## Connect from Another Machine

The run command binds to loopback. For a remote browser, set `HOST` to a reachable address on the ROS machine and use that address in the WebSocket URL. Only expose the demo on a trusted network. WSL2 usually forwards the port to Windows, so try the loopback URL first.