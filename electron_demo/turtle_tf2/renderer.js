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

const { ipcRenderer } = require('electron');

// 3D Scene variables
let scene, camera, renderer, controls;
let turtles = {};
let frames = {};
let coordinateFrames = {};
let turtlePoses = {}; // Track turtle poses for following logic

// Transform tracking
let activeTransforms = new Map();
let transformHistory = [];

// Demo state
let demoState = {
  rosConnected: false,
  turtlesimRunning: false,
  activeNodes: 0,
  totalNodes: 5,
};

// Keyboard control state
let keyState = {
  w: false,
  a: false,
  s: false,
  d: false,
};

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', function () {
  initializeScene();
  setupEventListeners();
  setupKeyboardControls();
  setupROSListeners();
  updateStatus();

  // Don't automatically hide loading screen - wait for ROS2 initialization
}); // Initialize Three.js scene
function initializeScene() {
  const container = document.getElementById('visualization-container');
  const canvas = document.getElementById('three-canvas');

  // Scene setup
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a2e);

  // Camera setup
  camera = new THREE.PerspectiveCamera(
    75,
    container.clientWidth / container.clientHeight,
    0.1,
    1000
  );
  camera.position.set(15, 15, 15);
  camera.lookAt(0, 0, 0);

  // Renderer setup
  renderer = new THREE.WebGLRenderer({
    canvas: canvas,
    antialias: true,
    alpha: true,
  });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  // Controls setup
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxDistance = 50;
  controls.minDistance = 5;

  // Lighting
  const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
  scene.add(ambientLight);

  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight.position.set(10, 10, 5);
  directionalLight.castShadow = true;
  directionalLight.shadow.mapSize.width = 2048;
  directionalLight.shadow.mapSize.height = 2048;
  scene.add(directionalLight);

  // Create ground plane
  createGroundPlane();

  // Create world coordinate frame
  createCoordinateFrame(
    'world',
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 0, w: 1 },
    3
  );

  // Start render loop
  animate();

  // Handle window resize
  window.addEventListener('resize', onWindowResize);
}

function createGroundPlane() {
  const geometry = new THREE.PlaneGeometry(20, 20, 20, 20);
  const material = new THREE.MeshLambertMaterial({
    color: 0x2a2a3e,
    wireframe: false,
    transparent: true,
    opacity: 0.8,
  });

  const plane = new THREE.Mesh(geometry, material);
  plane.rotation.x = -Math.PI / 2;
  plane.position.y = -0.1;
  plane.receiveShadow = true;
  scene.add(plane);

  // Add grid
  const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x333333);
  gridHelper.position.y = 0;
  scene.add(gridHelper);
}

function createTurtle(name, position, rotation, color) {
  const group = new THREE.Group();

  // Turtle body (ellipse)
  const bodyGeometry = new THREE.SphereGeometry(0.3, 16, 8);
  bodyGeometry.scale(1, 0.6, 1.2);
  const bodyMaterial = new THREE.MeshLambertMaterial({ color: color });
  const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
  body.castShadow = true;
  group.add(body);

  // Turtle head
  const headGeometry = new THREE.SphereGeometry(0.15, 8, 8);
  const headMaterial = new THREE.MeshLambertMaterial({ color: color });
  const head = new THREE.Mesh(headGeometry, headMaterial);
  head.position.set(0.4, 0, 0);
  head.castShadow = true;
  group.add(head);

  // Turtle shell pattern
  const shellGeometry = new THREE.CylinderGeometry(0.25, 0.25, 0.1, 8);
  const shellMaterial = new THREE.MeshLambertMaterial({
    color: new THREE.Color(color).multiplyScalar(0.7),
  });
  const shell = new THREE.Mesh(shellGeometry, shellMaterial);
  shell.position.y = 0.2;
  shell.castShadow = true;
  group.add(shell);

  // Position and rotate
  group.position.set(position.x, position.y + 0.2, position.z);
  group.rotation.y = rotation;

  // Add coordinate frame for turtle
  const frameGroup = createCoordinateFrame(
    name,
    position,
    { x: 0, y: 0, z: Math.sin(rotation / 2), w: Math.cos(rotation / 2) },
    1
  );
  group.add(frameGroup);

  scene.add(group);
  turtles[name] = group;

  return group;
}

function createCoordinateFrame(name, position, quaternion, scale = 1) {
  const group = new THREE.Group();

  // X axis (red)
  const xGeometry = new THREE.CylinderGeometry(
    0.02 * scale,
    0.02 * scale,
    1 * scale
  );
  const xMaterial = new THREE.MeshLambertMaterial({ color: 0xff0000 });
  const xAxis = new THREE.Mesh(xGeometry, xMaterial);
  xAxis.rotation.z = -Math.PI / 2;
  xAxis.position.x = 0.5 * scale;
  group.add(xAxis);

  // X arrow
  const xArrowGeometry = new THREE.ConeGeometry(0.05 * scale, 0.2 * scale, 8);
  const xArrow = new THREE.Mesh(xArrowGeometry, xMaterial);
  xArrow.rotation.z = -Math.PI / 2;
  xArrow.position.x = 1 * scale;
  group.add(xArrow);

  // Y axis (green)
  const yGeometry = new THREE.CylinderGeometry(
    0.02 * scale,
    0.02 * scale,
    1 * scale
  );
  const yMaterial = new THREE.MeshLambertMaterial({ color: 0x00ff00 });
  const yAxis = new THREE.Mesh(yGeometry, yMaterial);
  yAxis.position.y = 0.5 * scale;
  group.add(yAxis);

  // Y arrow
  const yArrowGeometry = new THREE.ConeGeometry(0.05 * scale, 0.2 * scale, 8);
  const yArrow = new THREE.Mesh(yArrowGeometry, yMaterial);
  yArrow.position.y = 1 * scale;
  group.add(yArrow);

  // Z axis (blue)
  const zGeometry = new THREE.CylinderGeometry(
    0.02 * scale,
    0.02 * scale,
    1 * scale
  );
  const zMaterial = new THREE.MeshLambertMaterial({ color: 0x0000ff });
  const zAxis = new THREE.Mesh(zGeometry, zMaterial);
  zAxis.rotation.x = Math.PI / 2;
  zAxis.position.z = 0.5 * scale;
  group.add(zAxis);

  // Z arrow
  const zArrowGeometry = new THREE.ConeGeometry(0.05 * scale, 0.2 * scale, 8);
  const zArrow = new THREE.Mesh(zArrowGeometry, zMaterial);
  zArrow.rotation.x = Math.PI / 2;
  zArrow.position.z = 1 * scale;
  group.add(zArrow);

  // Position the frame
  group.position.set(position.x, position.y, position.z);

  // Apply rotation from quaternion
  const euler = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
  );
  group.rotation.copy(euler);

  coordinateFrames[name] = group;
  scene.add(group);

  return group;
}

function createFrame(name, position, quaternion, color, scale = 0.5) {
  const group = new THREE.Group();

  // Frame marker (small sphere)
  const geometry = new THREE.SphereGeometry(0.1 * scale, 8, 8);
  const material = new THREE.MeshLambertMaterial({ color: color });
  const marker = new THREE.Mesh(geometry, material);
  marker.castShadow = true;
  group.add(marker);

  // Add coordinate axes
  const frameAxes = createCoordinateFrame(
    name + '_axes',
    { x: 0, y: 0, z: 0 },
    quaternion,
    scale
  );
  group.add(frameAxes);

  // Position the frame
  group.position.set(position.x, position.y, position.z);

  frames[name] = group;
  scene.add(group);

  return group;
}

function updateTurtlePose(name, pose) {
  // Store the pose for following logic
  turtlePoses[name] = pose;

  if (!turtles[name]) {
    // Create turtle if it doesn't exist
    const color = name === 'turtle1' ? 0x00ff00 : 0x0088ff;
    createTurtle(name, { x: pose.x, y: 0, z: pose.y }, pose.theta, color);
  } else {
    // Update existing turtle
    turtles[name].position.set(pose.x, 0.2, pose.y);
    turtles[name].rotation.y = pose.theta;
  }

  // Update coordinate frame
  if (coordinateFrames[name]) {
    coordinateFrames[name].position.set(pose.x, 0, pose.y);
    coordinateFrames[name].rotation.y = pose.theta;
  }
}

function updateFrame(name, transform) {
  const position = transform.transform.translation;
  const rotation = transform.transform.rotation;

  if (!frames[name]) {
    // Determine color based on frame name
    let color = 0xffffff;
    if (name.includes('static')) color = 0xff4444;
    else if (name.includes('dynamic')) color = 0xffaa00;
    else if (name.includes('fixed')) color = 0xaa44ff;

    createFrame(name, position, rotation, color);
  } else {
    // Update existing frame
    frames[name].position.set(position.x, position.y, position.z);

    const euler = new THREE.Euler().setFromQuaternion(
      new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
    );
    frames[name].rotation.copy(euler);
  }
}

function setupEventListeners() {
  // Control buttons
  document.getElementById('spawn-turtle1').addEventListener('click', () => {
    ipcRenderer.send('spawn-turtle-request', {
      name: 'turtle1',
      x: 5.5,
      y: 5.5,
      theta: 0,
    });
  });

  document.getElementById('spawn-turtle2').addEventListener('click', () => {
    ipcRenderer.send('spawn-turtle-request', {
      name: 'turtle2',
      x: 4.0,
      y: 2.0,
      theta: 0,
    });
  });

  document.getElementById('stop-all').addEventListener('click', () => {
    ipcRenderer.send('turtle-velocity-command', {
      turtle: 'turtle1',
      velocity: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
    });
    ipcRenderer.send('turtle-velocity-command', {
      turtle: 'turtle2',
      velocity: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
    });
  });

  document.getElementById('start-demo').addEventListener('click', () => {
    startDemo();
  });

  document.getElementById('reset-demo').addEventListener('click', () => {
    resetDemo();
  });

  // Frame toggles
  document.getElementById('toggle-static').addEventListener('click', () => {
    toggleFrameVisibility('carrot1_static');
  });

  document.getElementById('toggle-dynamic').addEventListener('click', () => {
    toggleFrameVisibility('carrot1_dynamic');
  });

  document.getElementById('toggle-fixed').addEventListener('click', () => {
    toggleFrameVisibility('carrot1_fixed');
  });
}

// Setup keyboard controls for turtle movement
function setupKeyboardControls() {
  // Track key state for WASD keys only
  document.addEventListener('keydown', (event) => {
    // Only handle WASD keys (case-insensitive)
    const key = event.key.toLowerCase();
    if (key in keyState) {
      keyState[key] = true;
      event.preventDefault();
    }
  });

  document.addEventListener('keyup', (event) => {
    // Only handle WASD keys (case-insensitive)
    const key = event.key.toLowerCase();
    if (key in keyState) {
      keyState[key] = false;
      event.preventDefault();
    }
  });

  // Send movement commands based on key state
  setInterval(() => {
    if (demoState.rosConnected) {
      sendTurtleCommand();
    }
  }, 100); // Send commands at 10Hz
}

// Send turtle movement command based on keyboard input
function sendTurtleCommand() {
  let linear_x = 0;
  let angular_z = 0;
  const speed = 2.0; // Linear speed
  const turn_speed = 2.0; // Angular speed

  // Check for forward/backward movement (W/S keys only)
  if (keyState.w) {
    linear_x = speed;
  } else if (keyState.s) {
    linear_x = -speed;
  }

  // Check for rotation (A/D keys only)
  if (keyState.a) {
    angular_z = turn_speed;
  } else if (keyState.d) {
    angular_z = -turn_speed;
  }

  // Send command if any movement is detected
  if (linear_x !== 0 || angular_z !== 0) {
    ipcRenderer.send('turtle-cmd-vel', {
      linear: { x: linear_x, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angular_z },
    });
  }
}

function setupROSListeners() {
  // Listen for ROS2 initialization status
  ipcRenderer.on('ros-initialization-status', (event, data) => {
    const loadingScreen = document.getElementById('loading-screen');
    const loadingText = loadingScreen.querySelector('div:last-child');

    switch (data.status) {
      case 'initializing':
        demoState.rosConnected = false;
        loadingText.textContent = data.message;
        break;
      case 'progress':
        demoState.rosConnected = false;
        loadingText.textContent = data.message;
        break;
      case 'ready':
        demoState.rosConnected = true;
        demoState.activeNodes = 5;
        loadingText.textContent = data.message;
        // Hide loading screen after successful initialization
        setTimeout(() => {
          loadingScreen.classList.add('hidden');
        }, 1000);
        break;
      case 'error':
        demoState.rosConnected = false;
        loadingText.textContent = data.message;
        loadingText.style.color = '#ff4444';
        break;
    }
    updateStatus();
  });

  // Listen for turtle pose updates
  ipcRenderer.on('turtle-pose-update', (event, data) => {
    updateTurtlePose(data.name, data.pose);
    updateTransformList(data.transform);
    demoState.turtlesimRunning = true;
    updateStatus();
  });

  // Listen for TF transform updates
  ipcRenderer.on('tf-transform-update', (event, transform) => {
    updateTransformList(transform);
  });

  // Listen for static transform updates
  ipcRenderer.on('static-transform-update', (event, transform) => {
    updateFrame(transform.child_frame_id, transform);
    updateTransformList(transform);
  });

  // Listen for dynamic transform updates
  ipcRenderer.on('dynamic-transform-update', (event, transform) => {
    updateFrame(transform.child_frame_id, transform);
    updateTransformList(transform);
  });

  // Listen for fixed transform updates
  ipcRenderer.on('fixed-transform-update', (event, transform) => {
    updateFrame(transform.child_frame_id, transform);
    updateTransformList(transform);
  });

  // Listen for turtle spawned events
  ipcRenderer.on('turtle-spawned', (event, data) => {
    console.log('Turtle spawned:', data);
    updateStatus();
  });

  // Listen for follow requests
  ipcRenderer.on('request-turtle-follow', (event) => {
    // Implement turtle2 following turtle1 logic
    if (turtlePoses['turtle1'] && turtlePoses['turtle2']) {
      const turtle1Pose = turtlePoses['turtle1'];
      const turtle2Pose = turtlePoses['turtle2'];

      // Calculate distance and angle to turtle1
      const dx = turtle1Pose.x - turtle2Pose.x;
      const dy = turtle1Pose.y - turtle2Pose.y;
      const distance = Math.sqrt(dx * dx + dy * dy);

      // Calculate angle to target
      const targetAngle = Math.atan2(dy, dx);
      const currentAngle = turtle2Pose.theta;

      // Calculate angular difference (handling wrap-around)
      let angleDiff = targetAngle - currentAngle;
      while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
      while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

      // Calculate velocities
      const linearVel = Math.min(2.0 * distance, 2.0); // Max linear velocity of 2.0
      const angularVel = 4.0 * angleDiff; // Proportional angular velocity

      // Only move if there's significant distance
      if (distance > 0.5) {
        // Send velocity command to turtle2
        ipcRenderer.send('turtle2-cmd-vel', {
          linear: { x: linearVel, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: angularVel },
        });

        console.log(
          `Turtle2 following: distance=${distance.toFixed(2)}, linear=${linearVel.toFixed(2)}, angular=${angularVel.toFixed(2)}`
        );
      } else {
        // Stop turtle2 when close enough
        ipcRenderer.send('turtle2-cmd-vel', {
          linear: { x: 0.0, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: 0.0 },
        });
      }
    }
  });
}
function updateTransformList(transform) {
  activeTransforms.set(transform.child_frame_id, {
    ...transform,
    lastUpdate: Date.now(),
  });

  const transformList = document.getElementById('transform-list');
  transformList.innerHTML = '';

  activeTransforms.forEach((tf, frameName) => {
    const item = document.createElement('div');
    item.className = 'transform-item';

    const pos = tf.transform.translation;
    const rot = tf.transform.rotation;

    item.innerHTML = `
      <div class="frame-name">${frameName}</div>
      <div class="coordinates">
        Pos: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})<br>
        Rot: (${rot.x.toFixed(3)}, ${rot.y.toFixed(3)}, ${rot.z.toFixed(3)}, ${rot.w.toFixed(3)})
      </div>
    `;

    transformList.appendChild(item);
  });

  // Update active transforms count
  document.getElementById('active-transforms').textContent =
    activeTransforms.size;
}

function updateStatus() {
  document.getElementById('ros-status').textContent = demoState.rosConnected
    ? 'Connected'
    : 'Connecting...';
  document.getElementById('ros-status').className = demoState.rosConnected
    ? 'status-value active'
    : 'status-value';

  document.getElementById('turtlesim-status').textContent =
    demoState.turtlesimRunning ? 'Running' : 'Waiting...';
  document.getElementById('turtlesim-status').className =
    demoState.turtlesimRunning ? 'status-value active' : 'status-value';

  document.getElementById('tf2-nodes-status').textContent =
    `${demoState.activeNodes}/${demoState.totalNodes}`;
}

function startDemo() {
  console.log('Starting turtle TF2 demo...');
  demoState.rosConnected = true;
  demoState.activeNodes = 5;
  updateStatus();

  // Create initial turtles
  if (!turtles['turtle1']) {
    createTurtle('turtle1', { x: 5.5, y: 0, z: 5.5 }, 0, 0x00ff00);
  }
}

function resetDemo() {
  console.log('Resetting demo...');

  // Clear all turtles
  Object.values(turtles).forEach((turtle) => {
    scene.remove(turtle);
  });
  turtles = {};

  // Clear all frames
  Object.values(frames).forEach((frame) => {
    scene.remove(frame);
  });
  frames = {};

  // Clear coordinate frames (except world)
  Object.entries(coordinateFrames).forEach(([name, frame]) => {
    if (name !== 'world') {
      scene.remove(frame);
      delete coordinateFrames[name];
    }
  });

  // Clear transforms
  activeTransforms.clear();
  document.getElementById('transform-list').innerHTML =
    '<div class="transform-item"><div class="frame-name">No transforms detected</div><div class="coordinates">Waiting for TF2 data...</div></div>';

  // Reset demo state
  demoState.turtlesimRunning = false;
  demoState.activeNodes = 0;
  updateStatus();
}

function toggleFrameVisibility(frameName) {
  if (frames[frameName]) {
    frames[frameName].visible = !frames[frameName].visible;
  }
}

function animate() {
  requestAnimationFrame(animate);

  // Update controls
  controls.update();

  // Render scene
  renderer.render(scene, camera);
}

function onWindowResize() {
  const container = document.getElementById('visualization-container');
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}
