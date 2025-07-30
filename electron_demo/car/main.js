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

let mainWindow;
let carControlNode;
let carControlPublisher;

function createWindow() {
  // Create the browser window.
  mainWindow = new BrowserWindow({
    width: 1000,
    height: 800,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
    },
    title: 'ROS2 Car Control Demo',
  });

  mainWindow.loadFile('index.html');
}

// Initialize ROS2 node for car control
async function initializeROS() {
  try {
    await rclnodejs.init();

    // Create node for car control
    carControlNode = rclnodejs.createNode('car_control_node');

    // Create publisher for car velocity commands (using geometry_msgs/Twist)
    carControlPublisher = carControlNode.createPublisher(
      'geometry_msgs/msg/Twist',
      'cmd_vel'
    );

    // Create subscription to listen to our own commands (for demo purposes)
    carControlNode.createSubscription(
      'geometry_msgs/msg/Twist',
      'cmd_vel',
      (msg) => {
        // Send velocity data to renderer process
        if (mainWindow) {
          mainWindow.webContents.send('velocity-received', {
            linear: {
              x: msg.linear.x,
              y: msg.linear.y,
              z: msg.linear.z,
            },
            angular: {
              x: msg.angular.x,
              y: msg.angular.y,
              z: msg.angular.z,
            },
          });
        }
      }
    );

    rclnodejs.spin(carControlNode);
    console.log('ROS2 car control node initialized successfully');
  } catch (error) {
    console.error('Failed to initialize ROS2:', error);
  }
}

// Handle joystick commands from renderer process
ipcMain.on('joystick-command', (event, command) => {
  if (carControlPublisher) {
    // Create Twist message based on joystick command
    const twist = {
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 },
    };

    // Map joystick commands to robot movement
    const speed = 1.0; // m/s
    const turnSpeed = 1.0; // rad/s

    switch (command) {
      case 'forward':
        twist.linear.x = speed;
        break;
      case 'backward':
        twist.linear.x = -speed;
        break;
      case 'left':
        twist.angular.z = turnSpeed;
        break;
      case 'right':
        twist.angular.z = -turnSpeed;
        break;
      case 'stop':
        // All values remain 0.0
        break;
    }

    // Publish the command
    carControlPublisher.publish(twist);
    console.log(`Published command: ${command}`, twist);
  }
});

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
app.whenReady().then(async () => {
  createWindow();
  await initializeROS();

  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

// Quit when all windows are closed, except on macOS.
app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') {
    if (carControlNode) {
      rclnodejs.shutdown();
    }
    app.quit();
  }
});

// Handle app termination
app.on('before-quit', () => {
  if (carControlNode) {
    rclnodejs.shutdown();
  }
});
