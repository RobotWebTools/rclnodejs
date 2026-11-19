# TypeScript Demos

Start with [topics](topics/README.md), then try [services](services/README.md) or [actions](actions/README.md).

## Setup

Use Node.js 24.11 or later in the 24.x line, npm 11.19.0, and a supported ROS 2 release. These demos build from the checkout and need the [source-development requirements](../../docs/BUILDING.md#requirements), not just the published package's Node.js 20.20.2 runtime minimum.

Install the interface package for your demo first: `std_msgs` for topics, `example_interfaces` for services, or `test_msgs` for actions.

From the repository root, run these commands. Replace `lyrical` with your distro:

```bash
source /opt/ros/lyrical/setup.bash
npm install
npm run build:dist
```

`npm install` prepares the native addon and generates ROS interfaces and TypeScript declarations. `npm run build:dist` bundles the JavaScript the demos import.

The demos link to this checkout (`file:../../..`). Follow your demo's run steps and source ROS in each terminal you use.

For the ROS 2 basics, see [Writing Your First rclnodejs Nodes](../../tutorials/writing-your-first-rclnodejs-nodes.md).
