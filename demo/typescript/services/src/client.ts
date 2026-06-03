/**
 * TypeScript Service Client Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 service client using TypeScript
 * with rclnodejs. It calls the AddTwoInts service with random numbers.
 *
 * It also demonstrates the two equivalent, fully type-safe ways to
 * identify a service type:
 *   - String name:   node.createClient('example_interfaces/srv/AddTwoInts', ...)
 *   - Service class: node.createClient(AddTwoInts, ...)
 *
 * This demo uses the class form (obtained from `rclnodejs.require(...)`):
 * TypeScript infers the request and the response-callback types directly
 * from the constructor, so no explicit `any` annotations are needed.
 */

import * as rclnodejs from 'rclnodejs';

const SERVICE_NAME = 'add_two_ints';
const REQUEST_INTERVAL = 3000000000n; // 3 seconds in nanoseconds

/**
 * Main service client function
 */
async function main(): Promise<void> {
  try {
    console.log('Starting TypeScript Service Client Demo...');

    // Initialize rclnodejs
    await rclnodejs.init();
    console.log('✓ rclnodejs initialized');

    // Create a node
    const node = new rclnodejs.Node('ts_client_demo');
    console.log(`✓ Created node: ${node.getFullyQualifiedName()}`);

    // Create a client for AddTwoInts using the service class (type-based
    // form). Obtain the constructor with `rclnodejs.require(...)` and pass it
    // directly; the request and response types are inferred from it. The
    // string form `node.createClient('example_interfaces/srv/AddTwoInts', ...)`
    // works identically and is equally type-safe.
    const AddTwoInts = rclnodejs.require('example_interfaces/srv/AddTwoInts');
    const client = node.createClient(AddTwoInts, SERVICE_NAME);
    console.log(`✓ Created service client: ${SERVICE_NAME}`);

    // Wait for the service to be available
    console.log('⏳ Waiting for service to be available...');
    const serviceAvailable = await client.waitForService(5000);

    if (!serviceAvailable) {
      console.error('❌ Service not available after 5 seconds');
      console.log('💡 Make sure the service server is running!');
      process.exit(1);
    }

    console.log('✓ Service is available');
    console.log('🚀 Starting to send requests. Press Ctrl+C to stop...\n');

    // Request counter
    let requestCount = 0;

    // Create a timer to send requests at regular intervals
    const timer = node.createTimer(REQUEST_INTERVAL, () => {
      try {
        requestCount++;

        // Generate random numbers for the request
        const a = Math.floor(Math.random() * 100);
        const b = Math.floor(Math.random() * 100);

        // Create a typed request message by instantiating the class.
        // `a` and `b` are typed (bigint) from the inferred request type.
        const request = new AddTwoInts.Request();
        request.a = BigInt(a);
        request.b = BigInt(b);

        console.log(`📞 [${requestCount}] Sending request: a=${a}, b=${b}`);
        console.log(`    Timestamp: ${new Date().toISOString()}`);

        // Send the request and wait for response (response is typed)
        client.sendRequest(request, (response) => {
          console.log(
            `📨 [${requestCount}] Received response: sum=${response.sum}`
          );
          console.log(
            `    Verification: ${a} + ${b} = ${Number(response.sum)} ✓`
          );
          console.log(`    Response time: ${new Date().toISOString()}\n`);
        });
      } catch (error) {
        console.error(`❌ [${requestCount}] Error sending request:`, error);
      }
    });

    console.log(
      `✓ Created timer with ${Number(REQUEST_INTERVAL / 1000000n)}ms interval`
    );

    // Spin the node to process callbacks
    rclnodejs.spin(node);
  } catch (error) {
    console.error('❌ Error in service client:', error);
    process.exit(1);
  }
}

// Handle shutdown gracefully
process.on('SIGINT', async () => {
  console.log('\n🛑 Shutting down service client...');
  try {
    await rclnodejs.shutdown();
    console.log('✓ Service client shutdown complete');
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
