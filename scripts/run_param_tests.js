
'use strict';

const fs = require('fs-extra');
const Mocha = require('mocha');
const os = require('os');
const path = require('path');

let rootDir = path.dirname(__dirname);
let msgPath = path.join(rootDir, 'test', 'rclnodejs_test_msgs');
process.env.AMENT_PREFIX_PATH = process.env.AMENT_PREFIX_PATH + path.delimiter + msgPath;

const includedCases = ['parameters_test.js'];

let mocha = new Mocha();
const testDir = path.join(__dirname, '../test/');

// eslint-disable-next-line
mocha.addFile(path.join(testDir, 'test-parameters.js'));
mocha.addFile(path.join(testDir, 'test-parameter-service.js'));
// mocha.addFile(path.join(testDir, 'run-node.js'));

mocha.run(function (failures) {
  process.on('exit', () => {
    process.exit(failures);
  });
});

