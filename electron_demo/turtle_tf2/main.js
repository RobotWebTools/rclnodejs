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
let turtleTf2Nodes = {};

// Quaternion from Euler helper function (from Python turtle_tf2_py)
function quaternionFromEuler(ai, aj, ak) {
  const ai_half = ai / 2.0;
  const aj_half = aj / 2.0;
  const ak_half = ak / 2.0;
  const ci = Math.cos(ai_half);
  const si = Math.sin(ai_half);
  const cj = Math.cos(aj_half);
  const sj = Math.sin(aj_half);
  const ck = Math.cos(ak_half);
  const sk = Math.sin(ak_half);
  const cc = ci * ck;
  const cs = ci * sk;
  const sc = si * ck;
  const ss = si * sk;

  return [
    cj * sc - sj * cs, // x
    cj * ss + sj * cc, // y
    cj * cs - sj * sc, // z
    cj * cc + sj * ss, // w
  ];
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1400,
    height: 900,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
    },
    title: 'ROS2 Turtle TF2 Demo with 3D Visualization',
  });

  mainWindow.loadFile('index.html');

  // Open DevTools for debugging (commented out for production)
  // mainWindow.webContents.openDevTools();
}

// Initialize ROS2 nodes for turtle TF2 demo
async function initializeROS() {
  try {
    console.log('Starting ROS2 initialization...');

    // Notify renderer that initialization is starting
    if (mainWindow) {
      mainWindow.webContents.send('ros-initialization-status', {
        status: 'initializing',
        message: 'Initializing ROS2 context...',
      });
    }

    await rclnodejs.init();
    console.log('ROS2 initialized successfully');

    if (mainWindow) {
      mainWindow.webContents.send('ros-initialization-status', {
        status: 'progress',
        message: 'Creating TF2 nodes...',
      });
    }

    // Create turtle TF2 broadcaster node
    console.log('Creating turtle TF2 broadcaster...');
    await createTurtleTf2Broadcaster();

    // Create turtle TF2 listener node
    console.log('Creating turtle TF2 listener...');
    await createTurtleTf2Listener();

    // Create static turtle TF2 broadcaster
    console.log('Creating static TF2 broadcaster...');
    await createStaticTurtleTf2Broadcaster();

    // Create dynamic frame TF2 broadcaster
    console.log('Creating dynamic frame TF2 broadcaster...');
    await createDynamicFrameTf2Broadcaster();

    // Create fixed frame TF2 broadcaster
    console.log('Creating fixed frame TF2 broadcaster...');
    await createFixedFrameTf2Broadcaster();

    console.log('All turtle TF2 nodes initialized successfully');

    // Notify renderer that initialization is complete
    if (mainWindow) {
      mainWindow.webContents.send('ros-initialization-status', {
        status: 'ready',
        message: 'ROS2 TF2 Demo ready!',
      });
    }
  } catch (error) {
    console.error('Failed to initialize ROS2:', error);

    // Notify renderer of initialization failure
    if (mainWindow) {
      mainWindow.webContents.send('ros-initialization-status', {
        status: 'error',
        message: `Failed to initialize ROS2: ${error.message}`,
      });
    }
  }
}

// Turtle TF2 Broadcaster - broadcasts turtle pose as transform
async function createTurtleTf2Broadcaster() {
  const node = rclnodejs.createNode('turtle_tf2_frame_publisher');

  // Create transform broadcaster
  const tfBroadcaster = node.createPublisher('tf2_msgs/msg/TFMessage', '/tf');

  // Create velocity publisher for turtle control
  const velocityPublisher = node.createPublisher(
    'geometry_msgs/msg/Twist',
    '/turtle1/cmd_vel'
  );
  turtleTf2Nodes.velocityPublisher = velocityPublisher;

  // Subscribe to turtle1 pose
  node.createSubscription('turtlesim/msg/Pose', '/turtle1/pose', (msg) => {
    const now = node.now();

    // Create transform message
    const transform = {
      header: {
        stamp: now,
        frame_id: 'world',
      },
      child_frame_id: 'turtle1',
      transform: {
        translation: {
          x: msg.x,
          y: msg.y,
          z: 0.0,
        },
        rotation: {
          x: 0.0,
          y: 0.0,
          z: Math.sin(msg.theta / 2.0),
          w: Math.cos(msg.theta / 2.0),
        },
      },
    };

    // Publish transform
    const tfMessage = {
      transforms: [transform],
    };

    tfBroadcaster.publish(tfMessage);

    // Send to renderer
    if (mainWindow) {
      mainWindow.webContents.send('turtle-pose-update', {
        name: 'turtle1',
        pose: {
          x: msg.x,
          y: msg.y,
          theta: msg.theta,
        },
        transform: {
          header: {
            stamp: {
              sec: now.sec,
              nanosec: now.nanosec,
            },
            frame_id: 'world',
          },
          child_frame_id: 'turtle1',
          transform: {
            translation: {
              x: msg.x,
              y: msg.y,
              z: 0.0,
            },
            rotation: {
              x: 0.0,
              y: 0.0,
              z: Math.sin(msg.theta / 2.0),
              w: Math.cos(msg.theta / 2.0),
            },
          },
        },
      });
    }
  });

  // Subscribe to turtle2 pose if it exists
  node.createSubscription('turtlesim/msg/Pose', '/turtle2/pose', (msg) => {
    const now = node.now();

    const transform = {
      header: {
        stamp: now,
        frame_id: 'world',
      },
      child_frame_id: 'turtle2',
      transform: {
        translation: {
          x: msg.x,
          y: msg.y,
          z: 0.0,
        },
        rotation: {
          x: 0.0,
          y: 0.0,
          z: Math.sin(msg.theta / 2.0),
          w: Math.cos(msg.theta / 2.0),
        },
      },
    };

    const tfMessage = {
      transforms: [transform],
    };

    tfBroadcaster.publish(tfMessage);

    if (mainWindow) {
      mainWindow.webContents.send('turtle-pose-update', {
        name: 'turtle2',
        pose: {
          x: msg.x,
          y: msg.y,
          theta: msg.theta,
        },
        transform: {
          header: {
            stamp: {
              sec: now.sec,
              nanosec: now.nanosec,
            },
            frame_id: 'world',
          },
          child_frame_id: 'turtle2',
          transform: {
            translation: {
              x: msg.x,
              y: msg.y,
              z: 0.0,
            },
            rotation: {
              x: 0.0,
              y: 0.0,
              z: Math.sin(msg.theta / 2.0),
              w: Math.cos(msg.theta / 2.0),
            },
          },
        },
      });
    }
  });

  rclnodejs.spin(node);
  turtleTf2Nodes.broadcaster = node;
}

// Turtle TF2 Listener - commands turtle2 to follow turtle1
async function createTurtleTf2Listener() {
  const node = rclnodejs.createNode('turtle_tf2_frame_listener');

  // Create velocity publisher for turtle2
  const velocityPublisher = node.createPublisher(
    'geometry_msgs/msg/Twist',
    '/turtle2/cmd_vel'
  );

  // Store turtle2 velocity publisher
  turtleTf2Nodes.turtle2VelocityPublisher = velocityPublisher;

  // Create service client to spawn turtle2
  const spawner = node.createClient('turtlesim/srv/Spawn', '/spawn');

  let turtleSpawningServiceReady = false;

  // Transform listener functionality
  const tfSubscriber = node.createSubscription(
    'tf2_msgs/msg/TFMessage',
    '/tf',
    (msg) => {
      // Process transforms for visualization
      msg.transforms.forEach((transform) => {
        if (mainWindow) {
          // Create a serializable version of the transform
          const serializableTransform = {
            header: {
              stamp: {
                sec: transform.header.stamp.sec,
                nanosec: transform.header.stamp.nanosec,
              },
              frame_id: transform.header.frame_id,
            },
            child_frame_id: transform.child_frame_id,
            transform: {
              translation: {
                x: transform.transform.translation.x,
                y: transform.transform.translation.y,
                z: transform.transform.translation.z,
              },
              rotation: {
                x: transform.transform.rotation.x,
                y: transform.transform.rotation.y,
                z: transform.transform.rotation.z,
                w: transform.transform.rotation.w,
              },
            },
          };
          mainWindow.webContents.send(
            'tf-transform-update',
            serializableTransform
          );
        }
      });
    }
  );

  // Timer to check for transforms and control turtle2
  const timer = node.createTimer(1000n, () => {
    // Wrap the async logic in a try-catch to handle promise rejections
    (async () => {
      try {
        // Don't automatically spawn turtle2 - let user control this via UI
        // This prevents service call errors if turtlesim is not running

        // Simple following logic (in real implementation, this would use TF lookup)
        // For demo purposes, we'll simulate the transform lookup behavior
        if (turtleTf2Nodes.turtle2Spawned) {
          // This is a simplified version - in real TF2, we'd lookup transforms
          // For the demo, we'll let the renderer handle the following logic
          if (mainWindow) {
            mainWindow.webContents.send('request-turtle-follow');
          }
        }
      } catch (error) {
        console.error('Timer callback error:', error);
      }
    })();
  });

  rclnodejs.spin(node);
  turtleTf2Nodes.listener = node;
}

// Static Turtle TF2 Broadcaster
async function createStaticTurtleTf2Broadcaster() {
  const node = rclnodejs.createNode('static_turtle_tf2_broadcaster');

  const staticTfBroadcaster = node.createPublisher(
    'tf2_msgs/msg/TFMessage',
    '/tf_static'
  );

  // Broadcast a static transform (carrot frame relative to world)
  const now = node.now();
  const staticTransform = {
    header: {
      stamp: now,
      frame_id: 'world',
    },
    child_frame_id: 'carrot1_static',
    transform: {
      translation: {
        x: 2.0,
        y: 3.0,
        z: 0.0,
      },
      rotation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
      },
    },
  };

  const staticTfMessage = {
    transforms: [staticTransform],
  };

  // Publish static transform once
  staticTfBroadcaster.publish(staticTfMessage);

  if (mainWindow) {
    // Create serializable version
    const serializableStaticTransform = {
      header: {
        stamp: {
          sec: now.sec,
          nanosec: now.nanosec,
        },
        frame_id: 'world',
      },
      child_frame_id: 'carrot1_static',
      transform: {
        translation: {
          x: 2.0,
          y: 3.0,
          z: 0.0,
        },
        rotation: {
          x: 0.0,
          y: 0.0,
          z: 0.0,
          w: 1.0,
        },
      },
    };
    mainWindow.webContents.send(
      'static-transform-update',
      serializableStaticTransform
    );
  }

  rclnodejs.spin(node);
  turtleTf2Nodes.staticBroadcaster = node;
}

// Dynamic Frame TF2 Broadcaster
async function createDynamicFrameTf2Broadcaster() {
  const node = rclnodejs.createNode('dynamic_frame_tf2_broadcaster');

  const tfBroadcaster = node.createPublisher('tf2_msgs/msg/TFMessage', '/tf');

  // Timer to broadcast dynamic transform
  const timer = node.createTimer(100n, () => {
    const now = node.now();

    // Use a more stable time calculation to avoid NaN
    const timeInSeconds = Date.now() / 1000.0; // Use JavaScript Date for stable timing
    const angle = timeInSeconds * 0.5; // Slower rotation (0.5 rad/sec instead of π rad/sec)

    console.log(
      '🕒 Dynamic frame time:',
      timeInSeconds.toFixed(2),
      'angle:',
      angle.toFixed(2)
    );

    const dynamicTransform = {
      header: {
        stamp: now,
        frame_id: 'carrot1_static',
      },
      child_frame_id: 'carrot1_dynamic',
      transform: {
        translation: {
          x: 2.0 * Math.sin(angle),
          y: 2.0 * Math.cos(angle),
          z: 0.0,
        },
        rotation: {
          x: 0.0,
          y: 0.0,
          z: 0.0,
          w: 1.0,
        },
      },
    };

    const tfMessage = {
      transforms: [dynamicTransform],
    };

    tfBroadcaster.publish(tfMessage);

    if (mainWindow) {
      // Create serializable version
      const serializableDynamicTransform = {
        header: {
          stamp: {
            sec: now.sec,
            nanosec: now.nanosec,
          },
          frame_id: 'carrot1_static',
        },
        child_frame_id: 'carrot1_dynamic',
        transform: {
          translation: {
            x: 2.0 * Math.sin(angle),
            y: 2.0 * Math.cos(angle),
            z: 0.0,
          },
          rotation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
          },
        },
      };

      console.log(
        '📤 Sending dynamic transform:',
        'x:',
        (2.0 * Math.sin(angle)).toFixed(2),
        'y:',
        (2.0 * Math.cos(angle)).toFixed(2)
      );

      mainWindow.webContents.send(
        'dynamic-transform-update',
        serializableDynamicTransform
      );
    }
  });

  rclnodejs.spin(node);
  turtleTf2Nodes.dynamicBroadcaster = node;
}

// Fixed Frame TF2 Broadcaster
async function createFixedFrameTf2Broadcaster() {
  const node = rclnodejs.createNode('fixed_frame_tf2_broadcaster');

  const tfBroadcaster = node.createPublisher('tf2_msgs/msg/TFMessage', '/tf');

  // Timer to broadcast fixed transform
  const timer = node.createTimer(100n, () => {
    const now = node.now();
    const fixedTransform = {
      header: {
        stamp: now,
        frame_id: 'turtle1',
      },
      child_frame_id: 'carrot1_fixed',
      transform: {
        translation: {
          x: 0.0,
          y: 2.0,
          z: 0.0,
        },
        rotation: {
          x: 0.0,
          y: 0.0,
          z: 0.0,
          w: 1.0,
        },
      },
    };

    const tfMessage = {
      transforms: [fixedTransform],
    };

    tfBroadcaster.publish(tfMessage);

    if (mainWindow) {
      // Create serializable version
      const serializableFixedTransform = {
        header: {
          stamp: {
            sec: now.sec,
            nanosec: now.nanosec,
          },
          frame_id: 'turtle1',
        },
        child_frame_id: 'carrot1_fixed',
        transform: {
          translation: {
            x: 0.0,
            y: 2.0,
            z: 0.0,
          },
          rotation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
          },
        },
      };
      mainWindow.webContents.send(
        'fixed-transform-update',
        serializableFixedTransform
      );
    }
  });

  rclnodejs.spin(node);
  turtleTf2Nodes.fixedBroadcaster = node;
}

// Handle turtle velocity commands from renderer
ipcMain.on('turtle-velocity-command', (event, data) => {
  const { turtle, velocity } = data;

  // Find the appropriate publisher (this would be created when needed)
  if (turtleTf2Nodes.velocityPublisher) {
    turtleTf2Nodes.velocityPublisher.publish(velocity);
  }
});

// Handle spawning requests from renderer
ipcMain.on('spawn-turtle-request', (event, data) => {
  const { name, x, y, theta } = data;

  if (turtleTf2Nodes.listener) {
    console.log(
      `Spawn request received: ${name || 'turtle2'} at (${x || 4}, ${y || 2}, ${theta || 0})`
    );

    // Use the existing spawner from the listener node or create one
    const listenerNode = turtleTf2Nodes.listener;
    let spawner = listenerNode._spawner;

    if (!spawner) {
      console.log('Creating new spawn service client...');
      spawner = listenerNode.createClient('turtlesim/srv/Spawn', '/spawn');
      listenerNode._spawner = spawner; // Cache it

      // Wait for service to be ready with timeout
      let retries = 0;
      const maxRetries = 10;

      const checkService = () => {
        if (spawner.isServiceServerAvailable()) {
          performSpawn();
        } else if (retries < maxRetries) {
          retries++;
          console.log(`Waiting for spawn service... (attempt ${retries})`);
          setTimeout(checkService, 200);
        } else {
          console.error('Turtlesim spawn service not available after waiting');
          if (mainWindow) {
            mainWindow.webContents.send('spawn-error', {
              message: 'Turtlesim spawn service not available after waiting',
            });
          }
        }
      };

      checkService();
    } else {
      performSpawn();
    }

    function performSpawn() {
      const xPos = x || 4.0;
      const yPos = y || 2.0;
      const angle = theta || 0.0;
      const turtleName = name || 'turtle2';

      console.log(`Spawning ${turtleName} at (${xPos}, ${yPos}, ${angle})`);

      // Use callback pattern as required by rclnodejs
      try {
        spawner.sendRequest(
          {
            x: xPos,
            y: yPos,
            theta: angle,
            name: turtleName,
          },
          (response) => {
            console.log('Spawn response:', response);

            if (response && response.name) {
              const spawnedName = response.name;
              console.log(`Successfully spawned ${spawnedName}`);

              // Set turtle spawned flag for following logic
              if (spawnedName === 'turtle2') {
                turtleTf2Nodes.turtle2Spawned = true;
              }

              if (mainWindow) {
                mainWindow.webContents.send('turtle-spawned', {
                  name: spawnedName,
                  x: xPos,
                  y: yPos,
                  theta: angle,
                });
              }
            } else {
              console.error('Invalid or empty response from spawn service');
              if (mainWindow) {
                mainWindow.webContents.send('spawn-error', {
                  message: 'Invalid response from spawn service',
                });
              }
            }
          }
        );
      } catch (error) {
        console.error(`Error sending spawn request: ${error.message}`);
        if (mainWindow) {
          mainWindow.webContents.send('spawn-error', {
            message: `Error sending spawn request: ${error.message}`,
          });
        }
      }
    }
  } else {
    console.error('Listener node not initialized');
    if (mainWindow) {
      mainWindow.webContents.send('spawn-error', {
        message: 'ROS2 listener node not ready',
      });
    }
  }
});

// Handle keyboard turtle control commands
ipcMain.on('turtle-cmd-vel', (event, data) => {
  if (turtleTf2Nodes.velocityPublisher) {
    const velocity = {
      linear: data.linear,
      angular: data.angular,
    };
    // Send velocity command to turtle1
    turtleTf2Nodes.velocityPublisher.publish(velocity);
  }
});

// Handle turtle2 following commands
ipcMain.on('turtle2-cmd-vel', (event, data) => {
  if (turtleTf2Nodes.turtle2VelocityPublisher) {
    const velocity = {
      linear: data.linear,
      angular: data.angular,
    };
    // Send velocity command to turtle2
    turtleTf2Nodes.turtle2VelocityPublisher.publish(velocity);
  }
});

app.whenReady().then(async () => {
  createWindow();

  // Wait for window to be ready to receive messages
  mainWindow.webContents.once('did-finish-load', async () => {
    await initializeROS();
  });

  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') {
    // Clean up ROS2 nodes
    Object.values(turtleTf2Nodes).forEach((node) => {
      try {
        rclnodejs.shutdown();
      } catch (error) {
        console.error('Error shutting down node:', error);
      }
    });
    app.quit();
  }
});

app.on('before-quit', () => {
  Object.values(turtleTf2Nodes).forEach((node) => {
    try {
      rclnodejs.shutdown();
    } catch (error) {
      console.error('Error shutting down node:', error);
    }
  });
});
