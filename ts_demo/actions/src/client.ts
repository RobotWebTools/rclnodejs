/**
 * TypeScript Action Client Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 action client using TypeScript
 * with rclnodejs. It sends Fibonacci calculation goals to an action server.
 */

import * as rclnodejs from 'rclnodejs';

const ACTION_NAME = 'fibonacci';
const FIBONACCI_ORDER = 10;

/**
 * Fibonacci Action Client Class
 */
class FibonacciActionClient {
  private node: rclnodejs.Node;
  private actionClient: rclnodejs.ActionClient<'test_msgs/action/Fibonacci'>;

  constructor(node: rclnodejs.Node) {
    this.node = node;

    // Start spinning the node to handle callbacks
    rclnodejs.spin(node);

    // Create action client for Fibonacci action
    this.actionClient = new rclnodejs.ActionClient(
      node,
      'test_msgs/action/Fibonacci',
      ACTION_NAME
    );
  }

  /**
   * Send a goal to the Fibonacci action server
   */
  async sendGoal(): Promise<void> {
    this.node.getLogger().info('Waiting for action server...');

    // Wait for the action server to be available
    await this.actionClient.waitForServer();
    this.node.getLogger().info('✓ Action server is available');

    // Get the Fibonacci action interface
    const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');

    // Create a new goal
    const goal = new Fibonacci.Goal();
    goal.order = FIBONACCI_ORDER;

    this.node
      .getLogger()
      .info(`Sending goal request for Fibonacci(${goal.order})...`);

    try {
      // Send the goal with feedback callback
      console.log(goal);
      const goalHandle = await this.actionClient.sendGoal(
        goal,
        (feedback: any) => this.feedbackCallback(feedback)
      );

      if (!goalHandle.isAccepted()) {
        this.node.getLogger().error('❌ Goal was rejected by the server');
        return;
      }

      this.node.getLogger().info('✓ Goal accepted by server');

      // Wait for the result
      const result = await goalHandle.getResult();

      if (goalHandle.isSucceeded()) {
        this.node
          .getLogger()
          .info(
            `✓ Goal succeeded! Fibonacci(${FIBONACCI_ORDER}) = ${result.sequence}`
          );
      } else {
        this.node
          .getLogger()
          .error(`❌ Goal failed with status: ${goalHandle.status}`);
      }
    } catch (error) {
      this.node.getLogger().error(`❌ Error during goal execution: ${error}`);
    } finally {
      // Shutdown rclnodejs
      rclnodejs.shutdown();
    }
  }

  /**
   * Callback function for receiving feedback from the action server
   */
  private feedbackCallback(feedback: any): void {
    this.node
      .getLogger()
      .info(`📊 Received feedback: [${feedback.sequence.join(', ')}]`);
  }
}

/**
 * Main function
 */
async function main(): Promise<void> {
  try {
    console.log('🚀 Starting TypeScript Action Client Demo...');

    // Initialize rclnodejs
    await rclnodejs.init();
    console.log('✓ rclnodejs initialized');

    // Create a node
    const node = new rclnodejs.Node('ts_action_client_demo');
    console.log(`✓ Created node: ${node.getFullyQualifiedName()}`);

    // Create action client and send goal
    const client = new FibonacciActionClient(node);
    await client.sendGoal();
  } catch (error) {
    console.error('❌ Error in action client demo:', error);
    process.exit(1);
  }
}

// Handle process termination gracefully
process.on('SIGINT', () => {
  console.log('\n🛑 Received SIGINT, shutting down gracefully...');
  rclnodejs.shutdown();
  process.exit(0);
});

// Run the main function
main().catch((error) => {
  console.error('❌ Fatal error:', error);
  process.exit(1);
});
