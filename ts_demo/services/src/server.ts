/**
 * TypeScript Service Server Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 service server using TypeScript
 * with rclnodejs. It provides an AddTwoInts service that adds two integers.
 */

import * as rclnodejs from 'rclnodejs';

const SERVICE_NAME = 'add_two_ints';

/**
 * Main service server function
 */
async function main(): Promise<void> {
  try {
    console.log('Starting TypeScript Service Server Demo...');

    // Initialize rclnodejs
    await rclnodejs.init();
    console.log('✓ rclnodejs initialized');

    // Create a node
    const node = new rclnodejs.Node('ts_server_demo');
    console.log(`✓ Created node: ${node.getFullyQualifiedName()}`);

    // Request counter
    let requestCount = 0;

    // Create a service server for AddTwoInts
    const service = node.createService(
      'example_interfaces/srv/AddTwoInts',
      SERVICE_NAME,
      (request: any, response: any) => {
        requestCount++;

        // Log the incoming request
        console.log(
          `🔢 [${requestCount}] Received request: a=${request.a}, b=${request.b}`
        );

        // Calculate the sum using response.template
        let result = response.template;
        result.sum = request.a + request.b;

        // Log the response
        console.log(`📤 [${requestCount}] Sending response: sum=${result.sum}`);
        console.log(
          `    Calculation: ${request.a} + ${request.b} = ${result.sum}`
        );
        console.log(`    Timestamp: ${new Date().toISOString()}\n`);

        // Send the response
        response.send(result);
      }
    );

    console.log(`✓ Created service server: ${SERVICE_NAME}`);
    console.log('🚀 Service server is running. Press Ctrl+C to stop...\n');

    // Spin the node to process callbacks
    rclnodejs.spin(node);
  } catch (error) {
    console.error('❌ Error in service server:', error);
    process.exit(1);
  }
}

// Handle shutdown gracefully
process.on('SIGINT', async () => {
  console.log('\n🛑 Shutting down service server...');
  try {
    await rclnodejs.shutdown();
    console.log('✓ Service server shutdown complete');
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
