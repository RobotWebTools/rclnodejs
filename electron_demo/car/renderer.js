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

// DOM elements
let currentCommandEl, linearXEl, angularZEl, topicNameEl;
let commandCountEl, lastReceivedCommandEl, carPositionEl;
let carEl;

// State variables
let commandCount = 0;
let carPosition = { x: 150, y: 150 }; // Center of 300x300 area
let carRotation = 0;

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', function () {
  initializeElements();
  setupEventListeners();
  setupROSListeners();

  const versionDiv = document.createElement('div');
  versionDiv.style.textAlign = 'center';
  versionDiv.style.padding = '10px';
  versionDiv.style.marginTop = '20px';
  versionDiv.innerText = 'Electron version: ' + process.versions.electron;
  document.querySelector('.container').appendChild(versionDiv);
});

function initializeElements() {
  // Control panel elements
  currentCommandEl = document.getElementById('current-command');
  linearXEl = document.getElementById('linear-x');
  angularZEl = document.getElementById('angular-z');
  topicNameEl = document.getElementById('topic-name');

  // Car display elements
  commandCountEl = document.getElementById('command-count');
  lastReceivedCommandEl = document.getElementById('last-received-command');
  carPositionEl = document.getElementById('car-position');
  carEl = document.getElementById('car');
}

function setupEventListeners() {
  // Joystick button event listeners
  document
    .getElementById('btn-forward')
    .addEventListener('click', () => sendCommand('forward'));
  document
    .getElementById('btn-backward')
    .addEventListener('click', () => sendCommand('backward'));
  document
    .getElementById('btn-left')
    .addEventListener('click', () => sendCommand('left'));
  document
    .getElementById('btn-right')
    .addEventListener('click', () => sendCommand('right'));
  document
    .getElementById('btn-stop')
    .addEventListener('click', () => sendCommand('stop'));

  // Keyboard controls
  document.addEventListener('keydown', handleKeyPress);
  document.addEventListener('keyup', handleKeyRelease);
}

function setupROSListeners() {
  // Listen for velocity messages from main process
  ipcRenderer.on('velocity-received', (event, velocity) => {
    handleVelocityReceived(velocity);
  });
}

function sendCommand(command) {
  // Send command to main process
  ipcRenderer.send('joystick-command', command);

  // Update UI immediately
  updateCommandDisplay(command, getVelocityForCommand(command));
}

function getVelocityForCommand(command) {
  const speed = 1.0;
  const turnSpeed = 1.0;

  switch (command) {
    case 'forward':
      return {
        linear: { x: speed, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
    case 'backward':
      return {
        linear: { x: -speed, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
    case 'left':
      return {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: turnSpeed },
      };
    case 'right':
      return {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -turnSpeed },
      };
    case 'stop':
    default:
      return { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
  }
}

function updateCommandDisplay(command, velocity) {
  currentCommandEl.textContent =
    command.charAt(0).toUpperCase() + command.slice(1);
  linearXEl.textContent = velocity.linear.x.toFixed(1);
  angularZEl.textContent = velocity.angular.z.toFixed(1);
}

function handleVelocityReceived(velocity) {
  commandCount++;

  // Determine command type from velocity
  let commandType = 'stop';
  if (velocity.linear.x > 0) commandType = 'forward';
  else if (velocity.linear.x < 0) commandType = 'backward';
  else if (velocity.angular.z > 0) commandType = 'left';
  else if (velocity.angular.z < 0) commandType = 'right';

  // Update car display
  updateCarDisplay(commandType, velocity);

  // Update status
  commandCountEl.textContent = commandCount;
  lastReceivedCommandEl.textContent =
    commandType.charAt(0).toUpperCase() + commandType.slice(1);
}

function updateCarDisplay(command, velocity) {
  // Move car based on command
  const moveDistance = 20;
  const rotationAngle = 15;

  switch (command) {
    case 'forward':
      carPosition.y = Math.max(30, carPosition.y - moveDistance);
      break;
    case 'backward':
      carPosition.y = Math.min(270, carPosition.y + moveDistance);
      break;
    case 'left':
      carRotation += rotationAngle;
      carPosition.x = Math.max(30, carPosition.x - moveDistance);
      break;
    case 'right':
      carRotation -= rotationAngle;
      carPosition.x = Math.min(270, carPosition.x + moveDistance);
      break;
    case 'stop':
      // No movement
      break;
  }

  // Apply transformations to car element
  carEl.style.left = `${carPosition.x}px`;
  carEl.style.top = `${carPosition.y}px`;
  carEl.style.transform = `translate(-50%, -50%) rotate(${carRotation}deg)`;

  // Update position display
  const positionDesc = getPositionDescription(carPosition.x, carPosition.y);
  carPositionEl.textContent = positionDesc;

  // Add visual feedback
  addMovementFeedback(command);
}

function getPositionDescription(x, y) {
  const centerX = 150;
  const centerY = 150;

  if (Math.abs(x - centerX) < 30 && Math.abs(y - centerY) < 30) {
    return 'Center';
  }

  let desc = '';
  if (y < centerY - 30) desc += 'North ';
  else if (y > centerY + 30) desc += 'South ';

  if (x < centerX - 30) desc += 'West';
  else if (x > centerX + 30) desc += 'East';

  return desc.trim() || 'Center';
}

function addMovementFeedback(command) {
  // Add temporary visual feedback
  carEl.style.boxShadow = getCommandColor(command);

  setTimeout(() => {
    carEl.style.boxShadow = '0 2px 5px rgba(0,0,0,0.2)';
  }, 200);
}

function getCommandColor(command) {
  switch (command) {
    case 'forward':
      return '0 0 15px #00ff00';
    case 'backward':
      return '0 0 15px #ff0000';
    case 'left':
      return '0 0 15px #0000ff';
    case 'right':
      return '0 0 15px #ffff00';
    case 'stop':
      return '0 0 15px #ff00ff';
    default:
      return '0 2px 5px rgba(0,0,0,0.2)';
  }
}

// Keyboard controls
function handleKeyPress(event) {
  switch (event.key.toLowerCase()) {
    case 'w':
    case 'arrowup':
      event.preventDefault();
      sendCommand('forward');
      break;
    case 's':
    case 'arrowdown':
      event.preventDefault();
      sendCommand('backward');
      break;
    case 'a':
    case 'arrowleft':
      event.preventDefault();
      sendCommand('left');
      break;
    case 'd':
    case 'arrowright':
      event.preventDefault();
      sendCommand('right');
      break;
    case ' ':
    case 'escape':
      event.preventDefault();
      sendCommand('stop');
      break;
  }
}

function handleKeyRelease(event) {
  // Auto-stop after key release (optional behavior)
  // Uncomment the following lines if you want auto-stop on key release
  /*
  if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(event.key.toLowerCase())) {
    setTimeout(() => sendCommand('stop'), 100);
  }
  */
}

// Reset car position (for demo purposes)
function resetCarPosition() {
  carPosition = { x: 150, y: 150 };
  carRotation = 0;
  updateCarDisplay('stop', {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  });
}

// Add reset button functionality (if needed)
document.addEventListener('dblclick', resetCarPosition);
