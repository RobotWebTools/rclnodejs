/**
 * TypeScript Subscriber Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 subscriber using TypeScript
 * with rclnodejs. It subscribes to string messages from the "ts_demo" topic.
 *
 * It also demonstrates the two equivalent, fully type-safe ways to
 * identify a message type:
 *   - String name:   node.createSubscription('std_msgs/msg/String', ...)
 *   - Message class: node.createSubscription(StringMsg, ...)
 *
 * With the class form (obtained from `rclnodejs.require(...)`), TypeScript
 * infers the callback's message type directly from the constructor, so no
 * explicit `message` annotation is needed.
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

    // Create a subscription for std_msgs/msg/String messages.
    //
    // Style A — string name. The callback message type is annotated
    // explicitly as rclnodejs.std_msgs.msg.String:
    const subscription = node.createSubscription(
      'std_msgs/msg/String',
      TOPIC_NAME,
      (message: rclnodejs.std_msgs.msg.String) => {
        receivedCount++;
        console.log(
          `📥 [A/string] [${receivedCount}] Received: "${message.data}"`
        );
      }
    );

    // Style B — message class. Obtain the constructor with
    // `rclnodejs.require(...)` and pass it directly. The `message`
    // parameter type is inferred as rclnodejs.std_msgs.msg.String, so no
    // explicit annotation is required. Both styles are equally type-safe.
    const StringMsg = rclnodejs.require('std_msgs/msg/String');
    node.createSubscription(StringMsg, TOPIC_NAME, (message) => {
      console.log(`📥 [B/class] Received: "${message.data}"`);
    });

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
