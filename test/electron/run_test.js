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

console.log('Launching Electron to run test_usability.js...');
const child = spawn(electron, [path.join(__dirname, 'test_usability.js')], {
  stdio: 'inherit',
  env: { ...process.env, ELECTRON_ENABLE_LOGGING: true },
});

child.on('close', (code) => {
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
