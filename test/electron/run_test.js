'use strict';

const path = require('path');
const { spawn } = require('child_process');

let electron;
try {
  electron = require('electron');
} catch (e) {
  console.error(
    'Electron module not found. Please install electron to run this test.'
  );
  process.exit(1);
}

let command = electron;
let args = [path.join(__dirname, 'test_usability.js'), '--no-sandbox'];

// Handle headless Linux environments (like CI) by using xvfb-run
if (process.platform === 'linux' && !process.env.DISPLAY) {
  console.log('No DISPLAY detected. Attempting to use xvfb-run...');
  command = 'xvfb-run';
  // -a: --auto-servernum (use explicit server number to avoid conflicts)
  args = ['-a', electron, ...args];
}

console.log('Launching Electron to run test_usability.js...');
const child = spawn(command, args, {
  stdio: 'inherit',
  env: { ...process.env, ELECTRON_ENABLE_LOGGING: true },
});

// Kill the child process if it doesn't exit within 30 seconds
const killTimeout = setTimeout(() => {
  console.error('Electron process did not exit in time, killing...');
  child.kill('SIGKILL');
}, 30 * 1000);

child.on('close', (code) => {
  clearTimeout(killTimeout);
  console.log(`Electron process exited with code ${code}`);
  if (code === 0) {
    console.log('Test Passed!');
    process.exit(0);
  } else {
    console.error('Test Failed!');
    process.exit(1);
  }
});

child.on('error', (err) => {
  console.error('Failed to start electron:', err);
  process.exit(1);
});
