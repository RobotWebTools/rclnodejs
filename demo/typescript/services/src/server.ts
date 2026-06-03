/**
 * TypeScript Service Server Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 service server using TypeScript
 * with rclnodejs. It provides an AddTwoInts service that adds two integers.
 *
 * It also demonstrates the two equivalent, fully type-safe ways to
 * identify a service type:
 *   - String name:   node.createService('example_interfaces/srv/AddTwoInts', ...)
 *   - Service class: node.createService(AddTwoInts, ...)
 *
 * This demo uses the class form (obtained from `rclnodejs.require(...)`):
 * TypeScript infers the callback's request and response types directly from
 * the constructor, so no explicit `any` annotations are needed.
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

    // Create a service server for AddTwoInts using the service class
    // (type-based form). Obtain the constructor with `rclnodejs.require(...)`
    // and pass it directly; the `request` and `response` parameters are
    // inferred from it. The string form
    // `node.createService('example_interfaces/srv/AddTwoInts', ...)` works
    // identically and is equally type-safe.
    const AddTwoInts = rclnodejs.require('example_interfaces/srv/AddTwoInts');
    const service = node.createService(
      AddTwoInts,
      SERVICE_NAME,
      (request, response) => {
        requestCount++;

        // Log the incoming request (request.a / request.b are typed)
        console.log(
          `🔢 [${requestCount}] Received request: a=${request.a}, b=${request.b}`
        );

        // Calculate the sum using response.template (typed response message)
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
