'use strict';

const assert = require('assert');
const rclnodejs = require('..');
const childProcess = require('child_process');

function forkOnlyRemoveRclnodejsHandlers() {
  const myHandler = () => {};
  process.on('SIGINT', myHandler);
  rclnodejs.removeSignalHandlers();
  if (myHandler in process.listeners('SIGINT')) {
    process.exitCode = 0;
  } else {
    process.exitCode = 1;
  }
}

function forkRemoveSignalHandlers() {
  rclnodejs.removeSignalHandlers();
  process.exitCode = process.listenerCount('SIGINT');
}

function forkDoPublish(context) {
  rclnodejs
    .init(context)
    .then(() => {
      const node = rclnodejs.createNode('test_pub', undefined, context);
      const publisher = node.createPublisher('std_msgs/msg/String', 'test');
      node.createTimer(100, () => {
        publisher.publish({ data: 'hello' });
      });
      rclnodejs.spin(node);
    })
    .catch(() => {
      process.exit(1);
    });
}

if (process.env['RCLNODEJS_TEST_FORK']) {
  switch (process.argv[2]) {
    case '--remove-signal-handlers':
      forkRemoveSignalHandlers();
      break;
    case '--only-remove-rclnodejs-handlers':
      forkOnlyRemoveRclnodejsHandlers();
      break;
    default:
      forkDoPublish(
        process.argv[2] === '--non-default-context' ? new rclnodejs.Context() : undefined
      );
  }
} else {
  describe('signal handler tests', () => {
    let child;

    beforeEach(async () => {
      await rclnodejs.init();
    });

    afterEach(() => {
      child.kill('SIGKILL');
      rclnodejs.shutdown();
    });

    it('gracefully shuts downs on SIGINT when only running default context', async () => {
      child = childProcess.fork(__filename, {
        env: { ...process.env, RCLNODEJS_TEST_FORK: true },
      });
      const node = rclnodejs.createNode('test_sub');
      node.createSubscription('std_msgs/msg/String', 'test', () => {
        if (!child.killed) {
          child.kill('SIGINT');
        }
      });
      rclnodejs.spin(node);
      await new Promise((res) => {
        child.on('close', (exitCode) => {
          assert.strictEqual(exitCode, 0);
          res();
        });
      });
    });

    it('gracefully shuts downs on SIGINT when running non-default context', async () => {
      child = childProcess.fork(__filename, ['--non-default-context'], {
        env: { ...process.env, RCLNODEJS_TEST_FORK: true },
      });
      const node = rclnodejs.createNode('test_sub');
      node.createSubscription('std_msgs/msg/String', 'test', () => {
        if (!child.killed) {
          child.kill('SIGINT');
        }
      });
      rclnodejs.spin(node);
      await new Promise((res) => {
        child.on('close', (exitCode) => {
          assert.strictEqual(exitCode, 0);
          res();
        });
      });
    });

    it('signal handlers are removed after call removeSignalHandlers', async () => {
      // Because signal handlers is a global event and is installed on import,
      // we need to fork a child process to ensure the current environment is not "tainted".
      child = childProcess.fork(__filename, ['--remove-signal-handlers'], {
        env: { ...process.env, RCLNODEJS_TEST_FORK: true },
      });
      await new Promise((res) => {
        child.on('close', (exitCode) => {
          assert.strictEqual(exitCode, 0);
          res();
        });
      });
    });

    it('removeSignalHandlers only removes rclnodejs signals', async () => {
      // Because signal handlers is a global event and is installed on import,
      // we need to fork a child process to ensure the current environment is not "tainted".
      child = childProcess.fork(__filename, ['--only-remove-rclnodejs-handlers'], {
        env: { ...process.env, RCLNODEJS_TEST_FORK: true },
      });
      await new Promise((res) => {
        child.on('close', (exitCode) => {
          assert.strictEqual(exitCode, 1);
          res();
        });
      });
    });
  });
}
