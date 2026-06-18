# ROS2 Car Control Demo with Electron and rclnodejs

This demo showcases how to use **rclnodejs** with **Electron** to create an interactive car control application. The demo features a virtual joystick that publishes ROS2 velocity commands and a car visualization that responds to those commands in real-time.

## 🚗 Features

### Joystick Control Panel

- **Directional Controls**: Up/Down/Left/Right buttons for car movement
- **Stop Button**: Emergency stop functionality
- **Keyboard Support**: WASD and arrow keys for control
- **Real-time Status**: Display current command and velocity values

### Car Visualization

- **Real-time Movement**: Car moves and rotates based on received commands
- **Position Tracking**: Shows current position (North/South/East/West/Center)
- **Visual Feedback**: Color-coded movement indicators
- **Command Counter**: Tracks total number of commands received

### ROS2 Integration

- **Topic**: `cmd_vel` (geometry_msgs/Twist)
- **Publisher**: Sends velocity commands
- **Subscriber**: Receives and displays velocity commands
- **Node Name**: `car_control_node`

## 🔧 Prerequisites

Before running this demo, ensure you have:

1. **ROS2 installed** (Humble, Jazzy, Kilted, Lyrical, or Rolling)
2. **Node.js** (version 20.20.2 or higher)
3. **rclnodejs built** and working

### ROS2 Setup

```bash
# Source your ROS2 installation
source /opt/ros/<your-ros-distro>/setup.bash

# Verify ROS2 is working
ros2 topic list
```

### Build rclnodejs

From the main rclnodejs directory:

```bash
npm install
npm run build
```

## 🚀 Installation & Running

1. **Navigate to the demo directory:**

   ```bash
   cd electron_car_demo
   ```

2. **Install dependencies:**

   ```bash
   npm install
   ```

3. **Native modules — no manual rebuild needed:**

   rclnodejs ships prebuilt binaries for Electron and selects the matching one at
   runtime from `ROS_DISTRO` + Linux codename + architecture, so just make sure
   ROS 2 is sourced (next step) before launching. Do not run `electron-rebuild`
   against rclnodejs — it recompiles from source and bypasses the prebuilt binary.
   The Forge `rebuildConfig` in `package.json` already excludes `rclnodejs` from
   the automatic rebuild step.

4. **Start the demo:**
   ```bash
   # Make sure ROS2 is sourced first
   source /opt/ros/<your-ros-distro>/setup.bash
   npm start
   ```

## 📦 Packaging for Distribution

You can package the application into a standalone folder using **Electron Forge**.

### 1. Build the Package

Run the following command to create a distributable executable:

```bash
npm run package
```

The output will be located in the `out/` directory.

**Technical Note on ASAR:** We enable ASAR but configure it to **unpack** the `rclnodejs` module. `rclnodejs` (v1.8.1+) requires file system access to generated code and native bindings, so we use the `asar.unpack` configuration in `package.json` to keep `rclnodejs` files accessible on disk while packing the rest of the application.

```json
"config": {
  "forge": {
    "packagerConfig": {
      "asar": {
        "unpack": "**/node_modules/rclnodejs/**"
      }
    }
  }
}
```

### 2. Create Installers (Optional)

To create a `.zip` file or other platform-specific installers (deb/rpm), run:

```bash
npm run make
```

**Note**: Creating DEB/RPM installers requires system tools like `dpkg` and `fakeroot`. For ZIP files, you need `zip`.

### 3. Running the Standalone Application

Even as a standalone application, **ROS 2 must be installed and sourced on the target machine** because `rclnodejs` links dynamically to the ROS 2 shared libraries.

```bash
# Source ROS2 environment
source /opt/ros/<your-ros-distro>/setup.bash

# Run the packaged executable
./out/rclnodejs-electron-car-demo-linux-x64/rclnodejs-electron-car-demo
```

![demo screenshot](./car-control-electron.gif)

## 🎮 How to Use

### Control Methods

#### Mouse Controls

- Click the directional buttons (↑↓←→) on the joystick
- Click the red **STOP** button to halt movement

#### Keyboard Controls

- **W** or **↑**: Move forward
- **S** or **↓**: Move backward
- **A** or **←**: Turn left
- **D** or **→**: Turn right
- **Space** or **Esc**: Stop

### Understanding the Interface

#### Left Panel - Joystick Control

- **Command**: Shows the current command being sent
- **Linear X**: Forward/backward velocity (m/s)
- **Angular Z**: Rotation velocity (rad/s)
- **Topic**: ROS2 topic name (`cmd_vel`)

#### Right Panel - Car Visualization

- **Received Commands**: Total count of commands received
- **Last Command**: Most recent command processed
- **Car Position**: Current position in the visualization area

## 🔍 Technical Details

### ROS2 Message Format

The demo uses `geometry_msgs/Twist` messages with the following structure:

```javascript
{
  linear: {
    x: 1.0,    // Forward/backward velocity (m/s)
    y: 0.0,    // Left/right velocity (typically 0 for cars)
    z: 0.0     // Up/down velocity (typically 0 for ground vehicles)
  },
  angular: {
    x: 0.0,    // Roll rate (typically 0 for cars)
    y: 0.0,    // Pitch rate (typically 0 for cars)
    z: 1.0     // Yaw rate (turning left/right in rad/s)
  }
}
```

### Command Mapping

| Command  | Linear X | Angular Z | Description            |
| -------- | -------- | --------- | ---------------------- |
| Forward  | +1.0     | 0.0       | Move forward at 1 m/s  |
| Backward | -1.0     | 0.0       | Move backward at 1 m/s |
| Left     | 0.0      | +1.0      | Turn left at 1 rad/s   |
| Right    | 0.0      | -1.0      | Turn right at 1 rad/s  |
| Stop     | 0.0      | 0.0       | Stop all movement      |

### Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Renderer      │    │   Main Process   │    │   ROS2 Network  │
│   (UI/Browser)  │    │   (Node.js)      │    │                 │
├─────────────────┤    ├──────────────────┤    ├─────────────────┤
│ • Joystick UI   │───▶│ • rclnodejs Node │───▶│ • cmd_vel Topic │
│ • Car Display   │◀───│ • Publisher      │    │ • Other Nodes   │
│ • Status Panel  │    │ • Subscriber     │◀───│ • Robot/Sim     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🧪 Testing with ROS2 Tools

You can test the demo using standard ROS2 command-line tools:

### Listen to Published Commands

```bash
# In a new terminal (with ROS2 sourced)
ros2 topic echo /cmd_vel
```

### Send Commands from Command Line

```bash
# Send a forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Send a turn left command
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}"

# Stop command
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### Check Active Topics

```bash
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic hz /cmd_vel
```

## 🤖 Integration with Real Robots

This demo can be easily connected to real robots or simulators:

### TurtleBot3 (Example)

```bash
# Launch TurtleBot3 simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# The demo will automatically control the robot via cmd_vel topic
```

### Custom Robot

Ensure your robot subscribes to the `/cmd_vel` topic with `geometry_msgs/Twist` messages.

## 🛠️ Customization

### Modify Velocity Values

Edit the `speed` and `turnSpeed` constants in `main.js`:

```javascript
const speed = 1.0; // Linear velocity (m/s)
const turnSpeed = 1.0; // Angular velocity (rad/s)
```

### Change Topic Name

Modify the topic name in `main.js`:

```javascript
// Change 'cmd_vel' to your desired topic name
carControlPublisher = carControlNode.createPublisher(
  'geometry_msgs/msg/Twist',
  'your_topic_name'
);
```

### Add More Commands

Extend the joystick commands by modifying the switch statement in `main.js` and adding corresponding UI elements.

## 🐛 Troubleshooting

### Common Issues

1. **"Failed to initialize ROS2" Error**
   - Ensure ROS2 is properly sourced before running npm start
   - Check that rclnodejs is built correctly

2. **Native module fails to load**
   - Ensure ROS 2 is sourced so rclnodejs can match its prebuilt binary
     (`ROS_DISTRO` must be set; prebuilt binaries are provided for Ubuntu).
   - As a last resort, force a from-source rebuild by setting
     `RCLNODEJS_FORCE_BUILD=1` before `npm start` (needs a compiler toolchain and
     network access).

3. **Topic Not Appearing**
   - Verify ROS2 daemon is running: `ros2 daemon status`
   - Check topic list: `ros2 topic list`

4. **Car Not Moving in UI**
   - Check browser console for JavaScript errors
   - Verify IPC communication between main and renderer processes

### Debug Mode

Add debug logging by modifying `main.js`:

```javascript
// Enable debug logging
console.log('Publishing command:', command, twist);
```

## 📚 Learning Resources

- [rclnodejs Documentation](https://github.com/RobotWebTools/rclnodejs)
- [ROS2 Tutorials](https://docs.ros.org/en/lyrical/Tutorials.html)
- [Electron Documentation](https://www.electronjs.org/docs)
- [geometry_msgs/Twist Documentation](https://docs.ros.org/en/lyrical/p/geometry_msgs/interfaces/msg/Twist.html)

## 📄 License

This demo is licensed under the Apache License 2.0, same as the main rclnodejs project.

## 🤝 Contributing

Feel free to submit issues and enhancement requests! This demo serves as both a functional example and a starting point for more complex ROS2 Electron applications.
