/**
 * TypeScript Publisher Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 publisher using TypeScript
 * with rclnodejs. It publishes string messages to the "ts_demo" topic.
 *
 * It also demonstrates the two equivalent, fully type-safe ways to
 * identify a message type:
 *   - String name:   node.createPublisher('std_msgs/msg/String', ...)
 *   - Message class: node.createPublisher(StringMsg, ...)
 *
 * With the class form (obtained from `rclnodejs.require(...)`), TypeScript
 * infers the publisher's message type directly from the constructor, so
 * `new StringMsg()` is typed without any explicit annotation.
 */

import * as rclnodejs from 'rclnodejs';

const TOPIC_NAME = 'ts_demo';
const PUBLISH_INTERVAL = 1000000000n; // nanoseconds

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

    // Style A — string name. Create a publisher for std_msgs/msg/String
    // messages, building each message with `createMessageObject`:
    const publisher = node.createPublisher('std_msgs/msg/String', TOPIC_NAME);
    console.log(`✓ Created publisher on topic: ${TOPIC_NAME}`);

    // Style B — message class. Obtain the constructor with
    // `rclnodejs.require(...)` and pass it directly. The publisher's message
    // type is inferred from the constructor, and `new StringMsg()` is typed
    // without any annotation. Both styles are equally type-safe.
    const StringMsg = rclnodejs.require('std_msgs/msg/String');
    const typedPublisher = node.createPublisher(StringMsg, TOPIC_NAME);

    // Message counter
    let messageCount = 0;

    // Create a timer to publish messages at regular intervals
    const timer = node.createTimer(PUBLISH_INTERVAL, () => {
      // Style A — build the message with createMessageObject:
      const message = rclnodejs.createMessageObject('std_msgs/msg/String');
      message.data = `Hello from TypeScript publisher! Message #${++messageCount} at ${new Date().toISOString()}`;
      publisher.publish(message);
      console.log(`📤 [A/string] Published: "${message.data}"`);

      // Style B — build the message by instantiating the class. The
      // `data` field is typed from the inferred message type.
      const typedMessage = new StringMsg();
      typedMessage.data = `Hello from typed publisher! Message #${messageCount}`;
      typedPublisher.publish(typedMessage);
      console.log(`📤 [B/class] Published: "${typedMessage.data}"`);
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
