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
const process = require('process');

// Three.js scene components
let scene, camera, renderer, controls;
let manipulator = {};
let isAnimating = false;
let messageCount = 0;
let lastMessageTime = 0;
let frequencyUpdateInterval;

// Joint angles (in radians)
let jointAngles = {
  joint1: 0.0,
  joint2: 0.0,
};

// Initialize the 3D scene
function initScene() {
  const versionDiv = document.createElement('div');
  versionDiv.style.position = 'absolute';
  versionDiv.style.bottom = '10px';
  versionDiv.style.right = '10px';
  versionDiv.style.color = 'white';
  versionDiv.style.fontFamily = 'Arial, sans-serif';
  versionDiv.style.zIndex = '1000';
  versionDiv.innerText = 'Electron version: ' + process.versions.electron;
  document.body.appendChild(versionDiv);

  const container = document.getElementById('canvas-container');

  // Scene setup
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x222222);

  // Camera setup
  camera = new THREE.PerspectiveCamera(
    75,
    window.innerWidth / window.innerHeight,
    0.1,
    1000
  );
  camera.position.set(5, 5, 5);
  camera.lookAt(0, 0, 0);

  // Renderer setup
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  // Lighting
  setupLighting();

  // Create the manipulator
  createManipulator();

  // Create ground plane
  createGround();

  // Setup camera controls (basic orbit controls)
  setupCameraControls();

  // Start render loop
  animate();
}

function setupLighting() {
  // Ambient light
  const ambientLight = new THREE.AmbientLight(0x404040, 0.3);
  scene.add(ambientLight);

  // Directional light (main light)
  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight.position.set(10, 10, 5);
  directionalLight.castShadow = true;
  directionalLight.shadow.mapSize.width = 2048;
  directionalLight.shadow.mapSize.height = 2048;
  scene.add(directionalLight);

  // Point light for additional illumination
  const pointLight = new THREE.PointLight(0xffffff, 0.5, 100);
  pointLight.position.set(-5, 5, 5);
  scene.add(pointLight);
}

function createManipulator() {
  // Base (fixed)
  const baseGeometry = new THREE.CylinderGeometry(0.8, 1.0, 0.5, 16);
  const baseMaterial = new THREE.MeshLambertMaterial({ color: 0x444444 });
  manipulator.base = new THREE.Mesh(baseGeometry, baseMaterial);
  manipulator.base.position.y = 0.25;
  manipulator.base.castShadow = true;
  manipulator.base.receiveShadow = true;
  scene.add(manipulator.base);

  // Joint 1 (rotating around Y-axis) - this will be a child of base
  const joint1Geometry = new THREE.SphereGeometry(0.3, 16, 16);
  const joint1Material = new THREE.MeshLambertMaterial({ color: 0xff4444 });
  manipulator.joint1 = new THREE.Mesh(joint1Geometry, joint1Material);
  manipulator.joint1.position.y = 0.5;
  manipulator.joint1.castShadow = true;
  scene.add(manipulator.joint1);

  // Link 1 (between joint1 and joint2)
  const link1Geometry = new THREE.CylinderGeometry(0.15, 0.15, 2.5, 16);
  const link1Material = new THREE.MeshLambertMaterial({ color: 0x4444ff });
  manipulator.link1 = new THREE.Mesh(link1Geometry, link1Material);
  manipulator.link1.position.set(0, 1.25, 0);
  manipulator.link1.castShadow = true;

  // Create a group for joint1 and its children (link1, joint2, link2, endEffector)
  manipulator.joint1Group = new THREE.Group();
  manipulator.joint1Group.position.y = 0.5; // Position at top of base
  manipulator.joint1Group.add(manipulator.joint1);
  manipulator.joint1Group.add(manipulator.link1);
  scene.add(manipulator.joint1Group);

  // Joint 2 (rotating around Z-axis)
  const joint2Geometry = new THREE.SphereGeometry(0.25, 16, 16);
  const joint2Material = new THREE.MeshLambertMaterial({ color: 0x44ff44 });
  manipulator.joint2 = new THREE.Mesh(joint2Geometry, joint2Material);
  manipulator.joint2.position.set(0, 2.5, 0);
  manipulator.joint2.castShadow = true;

  // Link 2 (end effector arm)
  const link2Geometry = new THREE.CylinderGeometry(0.1, 0.1, 2.0, 16);
  const link2Material = new THREE.MeshLambertMaterial({ color: 0xffff44 });
  manipulator.link2 = new THREE.Mesh(link2Geometry, link2Material);
  manipulator.link2.position.set(0, 1.0, 0);
  manipulator.link2.castShadow = true;

  // End effector
  const endEffectorGeometry = new THREE.BoxGeometry(0.3, 0.3, 0.3);
  const endEffectorMaterial = new THREE.MeshLambertMaterial({
    color: 0xff44ff,
  });
  manipulator.endEffector = new THREE.Mesh(
    endEffectorGeometry,
    endEffectorMaterial
  );
  manipulator.endEffector.position.set(0, 2.0, 0);
  manipulator.endEffector.castShadow = true;

  // Create a group for joint2 and its children (link2, endEffector)
  manipulator.joint2Group = new THREE.Group();
  manipulator.joint2Group.position.set(0, 2.5, 0); // Position at top of link1
  manipulator.joint2Group.add(manipulator.joint2);
  manipulator.joint2Group.add(manipulator.link2);
  manipulator.joint2Group.add(manipulator.endEffector);

  // Add joint2 group to joint1 group so it moves with joint1
  manipulator.joint1Group.add(manipulator.joint2Group);

  // Create coordinate axes for reference
  createCoordinateAxes();

  // Add visual markers for joints
  createJointMarkers();
}

function createCoordinateAxes() {
  const axesHelper = new THREE.AxesHelper(1);
  scene.add(axesHelper);
}

function createJointMarkers() {
  // Create 3D text labels and visual indicators for joints

  // Base marker - Fixed reference point
  const baseMarkerGeometry = new THREE.RingGeometry(1.2, 1.4, 16);
  const baseMarkerMaterial = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.8,
  });
  const baseMarker = new THREE.Mesh(baseMarkerGeometry, baseMarkerMaterial);
  baseMarker.rotation.x = -Math.PI / 2; // Rotate to lie flat on ground
  baseMarker.position.y = 0.01; // Just above ground
  scene.add(baseMarker);

  // Joint 1 (Base) indicator - Rotates with the arm
  const joint1IndicatorGeometry = new THREE.TorusGeometry(0.5, 0.05, 8, 16);
  const joint1IndicatorMaterial = new THREE.MeshBasicMaterial({
    color: 0xff0000,
    transparent: true,
    opacity: 0.7,
  });
  manipulator.joint1Indicator = new THREE.Mesh(
    joint1IndicatorGeometry,
    joint1IndicatorMaterial
  );
  manipulator.joint1Indicator.position.y = 0.5;
  manipulator.joint1Indicator.rotation.x = Math.PI / 2; // Rotate to be horizontal
  manipulator.joint1Group.add(manipulator.joint1Indicator);

  // Joint 2 (Elbow) indicator - Rotates with the elbow
  const joint2IndicatorGeometry = new THREE.TorusGeometry(0.4, 0.04, 8, 16);
  const joint2IndicatorMaterial = new THREE.MeshBasicMaterial({
    color: 0x00ff00,
    transparent: true,
    opacity: 0.7,
  });
  manipulator.joint2Indicator = new THREE.Mesh(
    joint2IndicatorGeometry,
    joint2IndicatorMaterial
  );
  manipulator.joint2Indicator.position.set(0, 2.5, 0);
  // This will rotate around Z-axis with joint2Group
  manipulator.joint2Group.add(manipulator.joint2Indicator);

  // Create arrow indicators to show rotation directions
  createRotationArrows();

  // Create text labels (using simple 3D geometry since we can't easily use text)
  createJointLabels();
}

function createRotationArrows() {
  // Joint 1 rotation arrow (around Y-axis)
  const arrowGeometry1 = new THREE.ConeGeometry(0.1, 0.3, 8);
  const arrowMaterial1 = new THREE.MeshBasicMaterial({ color: 0xff0000 });

  // Create multiple arrow cones to show circular motion
  for (let i = 0; i < 4; i++) {
    const arrow = new THREE.Mesh(arrowGeometry1, arrowMaterial1);
    const angle = (i / 4) * Math.PI * 2;
    arrow.position.set(Math.cos(angle) * 0.8, 0.7, Math.sin(angle) * 0.8);
    arrow.rotation.y = angle + Math.PI / 2; // Point in rotation direction
    arrow.rotation.z = Math.PI / 2; // Point horizontally
    manipulator.joint1Group.add(arrow);
  }

  // Joint 2 rotation arrow (around Z-axis)
  const arrowGeometry2 = new THREE.ConeGeometry(0.08, 0.25, 8);
  const arrowMaterial2 = new THREE.MeshBasicMaterial({ color: 0x00ff00 });

  for (let i = 0; i < 4; i++) {
    const arrow = new THREE.Mesh(arrowGeometry2, arrowMaterial2);
    const angle = (i / 4) * Math.PI * 2;
    arrow.position.set(Math.cos(angle) * 0.6, 2.5, Math.sin(angle) * 0.6);
    arrow.rotation.y = angle + Math.PI / 2; // Point in rotation direction
    arrow.rotation.z = Math.PI / 2; // Point horizontally
    manipulator.joint2Group.add(arrow);
  }
}

function createJointLabels() {
  // Create simple geometric shapes to represent labels

  // "BASE" label using boxes
  const labelMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });

  // BASE label at ground level
  const baseLabel = new THREE.Group();
  // Create letter-like shapes for "BASE" - simplified geometric representation
  const baseLabelGeometry = new THREE.BoxGeometry(0.8, 0.1, 0.1);
  const baseLabelMesh = new THREE.Mesh(baseLabelGeometry, labelMaterial);
  baseLabelMesh.position.set(2, 0.2, 0);
  scene.add(baseLabelMesh);

  // "JOINT 1" label near the base joint
  const joint1LabelGeometry = new THREE.BoxGeometry(0.6, 0.08, 0.08);
  const joint1LabelMesh = new THREE.Mesh(
    joint1LabelGeometry,
    new THREE.MeshBasicMaterial({ color: 0xff0000 })
  );
  joint1LabelMesh.position.set(1.5, 0.8, 0);
  manipulator.joint1Group.add(joint1LabelMesh);

  // "JOINT 2" label near the elbow joint
  const joint2LabelGeometry = new THREE.BoxGeometry(0.6, 0.08, 0.08);
  const joint2LabelMesh = new THREE.Mesh(
    joint2LabelGeometry,
    new THREE.MeshBasicMaterial({ color: 0x00ff00 })
  );
  joint2LabelMesh.position.set(1.2, 2.5, 0);
  manipulator.joint2Group.add(joint2LabelMesh);
}

function createGround() {
  const groundGeometry = new THREE.PlaneGeometry(20, 20);
  const groundMaterial = new THREE.MeshLambertMaterial({ color: 0x555555 });
  const ground = new THREE.Mesh(groundGeometry, groundMaterial);
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);
}

function setupCameraControls() {
  // Simple orbit controls using mouse
  let isDragging = false;
  let previousMousePosition = { x: 0, y: 0 };

  renderer.domElement.addEventListener('mousedown', (event) => {
    isDragging = true;
    previousMousePosition = { x: event.clientX, y: event.clientY };
  });

  renderer.domElement.addEventListener('mousemove', (event) => {
    if (isDragging) {
      const deltaMove = {
        x: event.clientX - previousMousePosition.x,
        y: event.clientY - previousMousePosition.y,
      };

      // Rotate camera around the scene
      const spherical = new THREE.Spherical();
      spherical.setFromVector3(camera.position);
      spherical.theta -= deltaMove.x * 0.01;
      spherical.phi += deltaMove.y * 0.01;
      spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));

      camera.position.setFromSpherical(spherical);
      camera.lookAt(0, 1, 0);

      previousMousePosition = { x: event.clientX, y: event.clientY };
    }
  });

  renderer.domElement.addEventListener('mouseup', () => {
    isDragging = false;
  });

  // Zoom with mouse wheel
  renderer.domElement.addEventListener('wheel', (event) => {
    const scale = event.deltaY > 0 ? 1.1 : 0.9;
    camera.position.multiplyScalar(scale);
    event.preventDefault();
  });
}

function updateManipulator() {
  if (manipulator.joint1Group && manipulator.joint2Group) {
    // Update joint1 (base rotation around Y-axis)
    manipulator.joint1Group.rotation.y = jointAngles.joint1;

    // Update joint2 (elbow rotation around Z-axis)
    manipulator.joint2Group.rotation.z = jointAngles.joint2;
  }
}

function animate() {
  requestAnimationFrame(animate);
  updateManipulator();
  renderer.render(scene, camera);
}

// Handle window resize
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// UI event handlers
document.addEventListener('DOMContentLoaded', () => {
  const replaceText = (selector, text) => {
    const element = document.getElementById(selector);
    if (element) element.innerText = text;
  };

  for (const type of ['chrome', 'node', 'electron']) {
    replaceText(`${type}-version`, process.versions[type] || 'Unavailable');
  }

  initScene();
  setupUIEventHandlers();
  startFrequencyMeasurement();
});

function setupUIEventHandlers() {
  const joint1Slider = document.getElementById('joint1-slider');
  const joint2Slider = document.getElementById('joint2-slider');
  const joint1Value = document.getElementById('joint1-value');
  const joint2Value = document.getElementById('joint2-value');
  const animateBtn = document.getElementById('animate-btn');
  const resetBtn = document.getElementById('reset-btn');

  // Joint sliders
  joint1Slider.addEventListener('input', (e) => {
    const degrees = parseFloat(e.target.value);
    const radians = (degrees * Math.PI) / 180;
    jointAngles.joint1 = radians;
    joint1Value.textContent = degrees.toFixed(1) + '°';

    // Send to main process
    ipcRenderer.send('set-joint-positions', {
      joint1: radians,
      joint2: jointAngles.joint2,
    });
  });

  joint2Slider.addEventListener('input', (e) => {
    const degrees = parseFloat(e.target.value);
    const radians = (degrees * Math.PI) / 180;
    jointAngles.joint2 = radians;
    joint2Value.textContent = degrees.toFixed(1) + '°';

    // Send to main process
    ipcRenderer.send('set-joint-positions', {
      joint1: jointAngles.joint1,
      joint2: radians,
    });
  });

  // Animation button
  animateBtn.addEventListener('click', () => {
    if (!isAnimating) {
      ipcRenderer.send('start-animation');
    } else {
      ipcRenderer.send('stop-animation');
    }
  });

  // Reset button
  resetBtn.addEventListener('click', () => {
    ipcRenderer.send('reset-position');
  });
}

function startFrequencyMeasurement() {
  // Update frequency display every second
  frequencyUpdateInterval = setInterval(() => {
    const now = Date.now();
    const timeDiff = (now - lastMessageTime) / 1000;
    const frequency = timeDiff > 0 ? messageCount / timeDiff : 0;

    document.getElementById('frequency').textContent =
      frequency.toFixed(1) + ' Hz';
    messageCount = 0;
    lastMessageTime = now;
  }, 1000);
}

// IPC event handlers
ipcRenderer.on('ros2-status', (event, status) => {
  const statusElement = document.getElementById('connection-status');
  if (status.connected) {
    statusElement.textContent = 'Connected';
    statusElement.className = 'status-connected';
  } else {
    statusElement.textContent = 'Disconnected';
    statusElement.className = 'status-disconnected';
    if (status.error) {
      console.error('ROS2 Error:', status.error);
    }
  }
});

ipcRenderer.on('joint-state-received', (event, data) => {
  messageCount++;
  document.getElementById('msg-count').textContent = messageCount.toString();

  // Update joint angles from received data
  if (data.names && data.positions) {
    for (let i = 0; i < data.names.length; i++) {
      if (data.names[i] === 'joint1') {
        jointAngles.joint1 = data.positions[i];
      } else if (data.names[i] === 'joint2') {
        jointAngles.joint2 = data.positions[i];
      }
    }

    // Update UI sliders
    updateUIFromJointAngles();
  }
});

ipcRenderer.on('joint-positions-updated', (event, positions) => {
  jointAngles.joint1 = positions.joint1;
  jointAngles.joint2 = positions.joint2;
  updateUIFromJointAngles();
});

ipcRenderer.on('animation-status', (event, status) => {
  isAnimating = status.running;
  const animateBtn = document.getElementById('animate-btn');
  animateBtn.textContent = isAnimating ? 'Stop Animation' : 'Start Animation';
});

function updateUIFromJointAngles() {
  const joint1Degrees = (jointAngles.joint1 * 180) / Math.PI;
  const joint2Degrees = (jointAngles.joint2 * 180) / Math.PI;

  document.getElementById('joint1-slider').value = joint1Degrees;
  document.getElementById('joint2-slider').value = joint2Degrees;
  document.getElementById('joint1-value').textContent =
    joint1Degrees.toFixed(1) + '°';
  document.getElementById('joint2-value').textContent =
    joint2Degrees.toFixed(1) + '°';
}
