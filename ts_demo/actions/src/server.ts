/**
 * TypeScript Action Server Demo for rclnodejs
 *
 * This demo shows how to create a ROS2 action server using TypeScript
 * with rclnodejs. It provides a Fibonacci calculation service.
 */

import * as rclnodejs from 'rclnodejs';

const ACTION_NAME = 'fibonacci';

/**
 * Fibonacci Action Server Class
 */
class FibonacciActionServer {
  private node: rclnodejs.Node;
  private actionServer: rclnodejs.ActionServer<'test_msgs/action/Fibonacci'>;

  constructor(node: rclnodejs.Node) {
    this.node = node;

    // Create action server for Fibonacci action
    this.actionServer = new rclnodejs.ActionServer(
      node,
      'test_msgs/action/Fibonacci',
      ACTION_NAME,
      this.executeCallback.bind(this),
      this.goalCallback.bind(this),
      undefined, // handleAcceptedCallback
      this.cancelCallback.bind(this)
    );

    this.node
      .getLogger()
      .info(`✓ Fibonacci action server created on topic '${ACTION_NAME}'`);
  }

  /**
   * Execute callback - performs the Fibonacci calculation
   */
  async executeCallback(
    goalHandle: rclnodejs.ServerGoalHandle<'test_msgs/action/Fibonacci'>
  ): Promise<any> {
    this.node
      .getLogger()
      .info(`🚀 Executing goal for Fibonacci(${goalHandle.request.order})`);

    const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');
    const feedbackMessage = new Fibonacci.Feedback();
    const sequence: number[] = [0, 1];

    // Check for invalid input
    if (goalHandle.request.order < 0) {
      this.node.getLogger().error('❌ Invalid order: must be non-negative');
      goalHandle.abort();
      return new Fibonacci.Result();
    }

    if (goalHandle.request.order === 0) {
      const result = new Fibonacci.Result();
      result.sequence = [0];
      goalHandle.succeed();
      this.node
        .getLogger()
        .info('✓ Goal completed immediately: Fibonacci(0) = [0]');
      return result;
    }

    // Start executing the action
    for (let i = 1; i < goalHandle.request.order; i++) {
      // Check if the goal has been canceled
      if (goalHandle.isCancelRequested) {
        goalHandle.canceled();
        this.node.getLogger().info('🛑 Goal was canceled');
        return new Fibonacci.Result();
      }

      // Update Fibonacci sequence
      sequence.push(sequence[i] + sequence[i - 1]);

      // Prepare feedback message
      feedbackMessage.sequence = [...sequence]; // Create a copy
      this.node
        .getLogger()
        .info(
          `📊 Publishing feedback: [${feedbackMessage.sequence.join(', ')}]`
        );

      // Publish the feedback
      goalHandle.publishFeedback(feedbackMessage);

      // Wait for 1 second to simulate computation time
      await new Promise((resolve) => setTimeout(resolve, 1000));
    }

    // Mark goal as succeeded
    goalHandle.succeed();

    // Prepare result message
    const result = new Fibonacci.Result();
    result.sequence = sequence;

    this.node
      .getLogger()
      .info(
        `✅ Goal completed! Fibonacci(${goalHandle.request.order}) = [${result.sequence.join(', ')}]`
      );

    return result;
  }

  /**
   * Goal callback - decides whether to accept or reject incoming goals
   *
   * Note: According to the type definition, this should receive ActionGoal<T>,
   * but the actual implementation may pass different types. Using 'any' to handle
   * this inconsistency and support ActionGoal<T>.
   */
  goalCallback(goal: any): rclnodejs.GoalResponse {
    const order = goal.order;

    this.node
      .getLogger()
      .info(`📥 Received goal request for Fibonacci(${order})`);

    // Accept goals with reasonable order values
    if (order > 50) {
      this.node
        .getLogger()
        .warn(`⚠️  Rejecting goal: order ${order} is too large (max: 50)`);
      return rclnodejs.GoalResponse.REJECT;
    }

    if (order < 0) {
      this.node
        .getLogger()
        .warn(`⚠️  Rejecting goal: order ${order} is negative`);
      return rclnodejs.GoalResponse.REJECT;
    }

    this.node.getLogger().info('✅ Goal accepted');
    return rclnodejs.GoalResponse.ACCEPT;
  }

  /**
   * Cancel callback - handles goal cancellation requests
   */
  cancelCallback(
    goalHandle:
      | rclnodejs.ServerGoalHandle<'test_msgs/action/Fibonacci'>
      | undefined
  ): rclnodejs.CancelResponse {
    this.node.getLogger().info('📥 Received cancel request');
    return rclnodejs.CancelResponse.ACCEPT;
  }
}

/**
 * Main function
 */
async function main(): Promise<void> {
  try {
    console.log('🚀 Starting TypeScript Action Server Demo...');

    // Initialize rclnodejs
    await rclnodejs.init();
    console.log('✓ rclnodejs initialized');

    // Create a node
    const node = new rclnodejs.Node('ts_action_server_demo');
    console.log(`✓ Created node: ${node.getFullyQualifiedName()}`);

    // Create action server
    const server = new FibonacciActionServer(node);
    console.log('✓ Fibonacci action server is ready to receive goals');
    console.log(
      '🔗 Use "ros2 action send_goal /fibonacci test_msgs/action/Fibonacci "{order: 10}" to test'
    );

    // Start spinning the node to handle incoming requests
    rclnodejs.spin(node);
  } catch (error) {
    console.error('❌ Error in action server demo:', error);
    process.exit(1);
  }
}

/**
 * Handle process termination gracefully
 */
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
