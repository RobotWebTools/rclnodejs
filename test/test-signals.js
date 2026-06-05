// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import assert from 'assert';
import rclnodejs from '../index.js';
import childProcess from 'child_process';
import os from 'os';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);

function forkOnlyRemoveRclnodejsHandlers() {
  const myHandler = () => {};
  process.on('SIGINT', myHandler);
  rclnodejs.removeSignalHandlers();
  if (process.listeners('SIGINT').includes(myHandler)) {
    process.exitCode = 0;
  } else {
    process.exitCode = 1;
  }
}

function forkRemoveSignalHandlers() {
  const listenerCount = process.listenerCount('SIGINT');
  rclnodejs.removeSignalHandlers();
  if (listenerCount - 1 === process.listenerCount('SIGINT')) {
    process.exitCode = 0;
  } else {
    process.exitCode = 1;
  }
}

function forkDoPublish(context) {
  rclnodejs
    .init(context)
    .then(() => {
      const node = rclnodejs.createNode('test_pub', undefined, context);
      const publisher = node.createPublisher('std_msgs/msg/String', 'test');
      node.createTimer(BigInt(100000), () => {
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
        process.argv[2] === '--non-default-context'
          ? new rclnodejs.Context()
          : undefined
      );
  }
} else {
  describe('signal handler tests', function () {
    let child;

    this.timeout(60 * 1000);

    beforeEach(async () => {
      await rclnodejs.init();
    });

    afterEach(() => {
      child.kill('SIGKILL');
      rclnodejs.shutdown();
    });

    // signals is not supported in windows, see https://nodejs.org/dist/latest-v14.x/docs/api/process.html#process_signal_events
    if (os.platform() !== 'win32') {
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
    }

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
      child = childProcess.fork(
        __filename,
        ['--only-remove-rclnodejs-handlers'],
        {
          env: { ...process.env, RCLNODEJS_TEST_FORK: true },
        }
      );
      await new Promise((res) => {
        child.on('close', (exitCode) => {
          assert.strictEqual(exitCode, 0);
          res();
        });
      });
    });
  });
}
