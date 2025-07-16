/**
 * TypeScript Publisher Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 publisher using TypeScript
 * with rclnodejs. It publishes string messages to the "ts_demo" topic.
 */

import * as rclnodejs from 'rclnodejs';

const TOPIC_NAME = 'ts_demo';
const PUBLISH_INTERVAL = 1000000000n; // milliseconds

/**
 * Main publisher function
 */
async function main(): Promise<void> {
  try {
    console.log('Starting TypeScript Publisher Demo...');

    // Initialize rclnodejs
    await rclnodejs.init();
    console.log('✓ rclnodejs initialized');

    // Create a node
    const node = new rclnodejs.Node('ts_publisher_demo');
    console.log(`✓ Created node: ${node.getFullyQualifiedName()}`);

    // Create a publisher for std_msgs/msg/String messages
    const publisher = node.createPublisher('std_msgs/msg/String', TOPIC_NAME);
    console.log(`✓ Created publisher on topic: ${TOPIC_NAME}`);

    // Message counter
    let messageCount = 0;

    // Create a timer to publish messages at regular intervals
    const timer = node.createTimer(PUBLISH_INTERVAL, () => {
      // Create a string message
      const message = rclnodejs.createMessageObject('std_msgs/msg/String');
      message.data = `Hello from TypeScript publisher! Message #${++messageCount} at ${new Date().toISOString()}`;

      // Publish the message
      publisher.publish(message);
      console.log(`📤 Published: "${message.data}"`);
    });

    console.log(`✓ Created timer with ${PUBLISH_INTERVAL}ms interval`);
    console.log('🚀 Publisher is running. Press Ctrl+C to stop...\n');

    // Spin the node to process callbacks
    rclnodejs.spin(node);
  } catch (error) {
    console.error('❌ Error in publisher:', error);
    process.exit(1);
  }
}

// Handle shutdown gracefully
process.on('SIGINT', async () => {
  console.log('\n🛑 Shutting down publisher...');
  try {
    await rclnodejs.shutdown();
    console.log('✓ Publisher shutdown complete');
    process.exit(0);
  } catch (error) {
    console.error('❌ Error during shutdown:', error);
    process.exit(1);
  }
});

// Run the main function
if (require.main === module) {
  main().catch((error) => {
    console.error('❌ Fatal error:', error);
    process.exit(1);
  });
}
