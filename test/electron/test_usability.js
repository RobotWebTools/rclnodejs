'use strict';

const rclnodejs = require('../../index.js');
const { app } = require('electron');

app.on('ready', () => {
  console.log('Electron version:', process.versions.electron);
  rclnodejs
    .init()
    .then(() => {
      console.log('rclnodejs initialized successfully.');
      const node = rclnodejs.createNode('test_electron_node');
      const publisher = node.createPublisher(
        'std_msgs/msg/String',
        'electron_test_topic'
      );

      const subscription = node.createSubscription(
        'std_msgs/msg/String',
        'electron_test_topic',
        (msg) => {
          if (msg.data === 'Hello from Electron') {
            console.log(
              'Successfully received message in Electron environment.'
            );
            rclnodejs.shutdown();
            app.quit();
            process.exit(0);
          }
        }
      );

      console.log('Publisher and Subscriber created.');

      // Publish repeatedly until received
      const interval = setInterval(() => {
        publisher.publish('Hello from Electron');
        console.log('Published message...');
      }, 100);

      // Set a timeout to fail the test
      setTimeout(() => {
        console.error('Test Failed: Timeout waiting for message.');
        clearInterval(interval);
        rclnodejs.shutdown();
        app.quit();
        process.exit(1);
      }, 10000);

      rclnodejs.spin(node);
    })
    .catch((e) => {
      console.error('Initialization failed:', e);
      app.quit();
      process.exit(1);
    });
});
