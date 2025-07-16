/**
 * TypeScript Subscriber Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 subscriber using TypeScript
 * with rclnodejs. It subscribes to string messages from the "ts_demo" topic.
 */

import * as rclnodejs from 'rclnodejs';

const TOPIC_NAME = 'ts_demo';

/**
 * Main subscriber function
 */
async function main(): Promise<void> {
  try {
    console.log('Starting TypeScript Subscriber Demo...');

    // Initialize rclnodejs
    await rclnodejs.init();
    console.log('✓ rclnodejs initialized');

    // Create a node
    const node = new rclnodejs.Node('ts_subscriber_demo');
    console.log(`✓ Created node: ${node.getFullyQualifiedName()}`);

    // Message counter
    let receivedCount = 0;

    // Create a subscription for std_msgs/msg/String messages
    const subscription = node.createSubscription(
      'std_msgs/msg/String',
      TOPIC_NAME,
      (message: rclnodejs.std_msgs.msg.String) => {
        receivedCount++;
        console.log(`📥 [${receivedCount}] Received: "${message.data}"`);
        console.log(`    Timestamp: ${new Date().toISOString()}`);
      }
    );

    console.log(`✓ Created subscription on topic: ${TOPIC_NAME}`);
    console.log('👂 Subscriber is listening. Press Ctrl+C to stop...\n');

    // Spin the node to process callbacks
    rclnodejs.spin(node);
  } catch (error) {
    console.error('❌ Error in subscriber:', error);
    process.exit(1);
  }
}

// Handle shutdown gracefully
process.on('SIGINT', async () => {
  console.log('\n🛑 Shutting down subscriber...');
  try {
    await rclnodejs.shutdown();
    console.log('✓ Subscriber shutdown complete');
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
