# Electron Demos

This directory contains interactive Electron applications that demonstrate various rclnodejs features with rich graphical interfaces. Each demo showcases how to integrate ROS2 functionality into desktop applications using web technologies.

## 📱 Available Demos

### 🔤 Basic Communication

- **[topics/](./topics/)** - Minimal publisher/subscriber demo
  - Basic ROS2 topic communication with simple UI
  - Publishes and subscribes to string messages
  - Ideal starting point for learning rclnodejs with Electron

### 🚗 Robot Control & Visualization

- **[car/](./car/)** - Interactive car control with joystick

  - Virtual joystick for velocity command publishing
  - Real-time car visualization responding to commands
  - Emergency stop functionality and directional controls

- **[manipulator/](./manipulator/)** - Two-joint robotic arm simulation
  - 3D robotic arm visualization using Three.js
  - Interactive joint controls with sliders
  - Automatic animation patterns and live feedback
  - ROS2 integration via `sensor_msgs/msg/JointState`

### 🌍 Advanced Robotics Applications

- **[turtle_tf2/](./turtle_tf2/)** - TF2 transforms and turtle simulation
  - Complete TF2 coordinate frame system implementation
  - Integration with turtlesim for turtle tracking
  - 3D WebGL visualization of coordinate transformations
  - Intelligent turtle following behavior

## Complexity Levels

| Demo            | Complexity        | Features                   | Best For              |
| --------------- | ----------------- | -------------------------- | --------------------- |
| **topics**      | ⭐ Beginner       | Basic pub/sub              | Learning fundamentals |
| **car**         | ⭐⭐ Intermediate | Control UI, visualization  | Interactive control   |
| **manipulator** | ⭐⭐⭐ Advanced   | 3D graphics, joint control | Robotics simulation   |
| **turtle_tf2**  | ⭐⭐⭐⭐ Expert   | TF2 system, complex logic  | Transform management  |

## 🛠️ Development Tips

### ROS2 Integration

- All demos use rclnodejs in the renderer process
- Node lifecycle management follows Electron best practices
- Proper cleanup on application exit

### UI Technologies

- **Electron**: Desktop application framework
- **Three.js**: 3D graphics and visualization
- **Modern CSS**: Responsive and accessible interfaces
- **Web APIs**: Canvas, WebGL, and DOM manipulation

### Performance Considerations

- Efficient ROS2 message handling
- Optimized rendering loops
- Memory management for long-running applications

## 📚 Related Resources

- [rclnodejs Tutorials](../tutorials/) - Core concepts and API usage
- [ROS2 Documentation](https://docs.ros.org/) - Official ROS2 reference
- [Electron Documentation](https://www.electronjs.org/docs) - Electron framework guide
