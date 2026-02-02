// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

const { app, BrowserWindow, ipcMain } = require('electron');
const rclnodejs = require('rclnodejs');

// Fix for WebGL/GPU rendering issues on Linux environment
// Forces software rendering (SwiftShader) if hardware acceleration fails
app.commandLine.appendSwitch('ignore-gpu-blocklist');
app.commandLine.appendSwitch('disable-gpu-sandbox');

let mainWindow;
let manipulatorNode;
let jointStatePublisher;
let jointStateSubscriber;
let isAnimating = false;
let animationInterval;

// Current joint positions (in radians)
let currentJointPositions = {
  joint1: 0.0, // Base rotation
  joint2: 0.0, // Elbow rotation
};

// Animation parameters
let animationTime = 0;
const animationSpeed = 0.02;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1200,
    height: 800,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
    },
  });

  mainWindow.loadFile('index.html');

  // Open DevTools in development
  if (process.env.NODE_ENV === 'development') {
    mainWindow.webContents.openDevTools();
  }
}

app.whenReady().then(async () => {
  createWindow();

  // Wait for window to be ready before initializing ROS2
  mainWindow.webContents.once('did-finish-load', async () => {
    try {
      await initializeROS2();
      console.log('ROS2 initialized successfully');
    } catch (error) {
      console.error('Failed to initialize ROS2:', error);
      // Send error status to renderer
      mainWindow.webContents.send('ros2-status', {
        connected: false,
        error: error.message,
      });
    }
  });

  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') {
    // Clean up ROS2 resources
    if (animationInterval) {
      clearInterval(animationInterval);
    }
    app.quit();
  }
});

async function initializeROS2() {
  console.log('Initializing ROS2...');

  // Initialize rclnodejs
  await rclnodejs.init();

  // Create the manipulator node
  manipulatorNode = rclnodejs.createNode('manipulator_demo');

  // Create publisher for joint states
  jointStatePublisher = manipulatorNode.createPublisher(
    'sensor_msgs/msg/JointState',
    '/joint_states'
  );

  // Create subscriber to receive joint state commands (for external control)
  jointStateSubscriber = manipulatorNode.createSubscription(
    'sensor_msgs/msg/JointState',
    '/joint_states',
    (msg) => {
      // Forward joint state data to renderer
      mainWindow.webContents.send('joint-state-received', {
        names: msg.name,
        positions: msg.position,
        velocities: msg.velocity || [],
        efforts: msg.effort || [],
      });
    }
  );

  // Start spinning the node
  rclnodejs.spin(manipulatorNode);

  // Send success status to renderer
  mainWindow.webContents.send('ros2-status', {
    connected: true,
    message: 'Connected to ROS2',
  });

  // Start publishing joint states
  startJointStatePublishing();
}

function startJointStatePublishing() {
  // Publish joint states at 10 Hz
  setInterval(() => {
    publishJointStates();
  }, 100); // 100ms = 10 Hz
}

function publishJointStates() {
  if (!jointStatePublisher) return;

  const now = Date.now() / 1000; // Convert to seconds

  const jointStateMsg = {
    header: {
      stamp: {
        sec: Math.floor(now),
        nanosec: Math.floor((now - Math.floor(now)) * 1e9),
      },
      frame_id: 'base_link',
    },
    name: ['joint1', 'joint2'],
    position: [currentJointPositions.joint1, currentJointPositions.joint2],
    velocity: [0.0, 0.0], // For simplicity, set velocities to zero
    effort: [0.0, 0.0], // For simplicity, set efforts to zero
  };

  jointStatePublisher.publish(jointStateMsg);
}

// IPC handlers for communication with renderer process
ipcMain.on('set-joint-positions', (event, positions) => {
  currentJointPositions.joint1 = positions.joint1;
  currentJointPositions.joint2 = positions.joint2;
});

ipcMain.on('start-animation', (event) => {
  if (!isAnimating) {
    isAnimating = true;
    animationTime = 0;

    animationInterval = setInterval(() => {
      animationTime += animationSpeed;

      // Create smooth sinusoidal motion for both joints
      currentJointPositions.joint1 = (Math.sin(animationTime) * Math.PI) / 3; // ±60 degrees
      currentJointPositions.joint2 =
        (Math.sin(animationTime * 1.5) * Math.PI) / 4; // ±45 degrees

      // Send updated positions to renderer
      event.sender.send('joint-positions-updated', {
        joint1: currentJointPositions.joint1,
        joint2: currentJointPositions.joint2,
      });
    }, 50); // 50ms = 20 Hz animation

    event.sender.send('animation-status', { running: true });
  }
});

ipcMain.on('stop-animation', (event) => {
  if (isAnimating) {
    isAnimating = false;
    if (animationInterval) {
      clearInterval(animationInterval);
      animationInterval = null;
    }
    event.sender.send('animation-status', { running: false });
  }
});

ipcMain.on('reset-position', (event) => {
  // Stop animation if running
  if (isAnimating) {
    isAnimating = false;
    if (animationInterval) {
      clearInterval(animationInterval);
      animationInterval = null;
    }
  }

  // Reset to zero position
  currentJointPositions.joint1 = 0.0;
  currentJointPositions.joint2 = 0.0;

  // Send updated positions to renderer
  event.sender.send('joint-positions-updated', {
    joint1: 0.0,
    joint2: 0.0,
  });

  event.sender.send('animation-status', { running: false });
});
