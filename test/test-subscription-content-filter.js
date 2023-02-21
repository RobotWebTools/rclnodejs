'use strict';

const childProcess = require('child_process');
const assert = require('assert');
const rclnodejs = require('../index.js');
const DistroUtils = rclnodejs.DistroUtils;
const RMWUtils = rclnodejs.RMWUtils;

function isContentFilteringSupported() {
  return (
    DistroUtils.getDistroId() >= DistroUtils.getDistroId('humble') &&
    RMWUtils.getRMWName() != RMWUtils.RMWNames.CYCLONEDDS
  );
}

describe('subscription content-filtering', function () {
  this.timeout(10 * 1000);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('isContentFilteringEnabled', function (done) {
    console.log('ctsupported: ', isContentFilteringSupported());

    let node = rclnodejs.createNode('string_subscription');
    let msgString = 'std_msgs/msg/String';
    let options = rclnodejs.Node.getDefaultOptions();
    options.contentFilter = {
      expression: "data = 'FilteredData'",
    };

    let subscription = node.createSubscription(
      msgString,
      'String_channel',
      options,
      (msg) => {}
    );
    assert.ok(
      subscription.isContentFilteringEnabled() === isContentFilteringSupported()
    );

    node.destroySubscription(subscription);
    subscription = node.createSubscription(
      msgString,
      'String_channel',
      (msg) => {}
    );
    assert.ok(!subscription.isContentFilteringEnabled());

    done();
  });

  it('no parameters', function (done) {
    if (!isContentFilteringSupported()) {
      this.skip();
    }

    let node = rclnodejs.createNode('string_subscription');
    let msgString = 'std_msgs/msg/String';
    let options = rclnodejs.Node.getDefaultOptions();
    options.contentFilter = {
      expression: "data = 'FilteredData'",
    };

    let msgCnt = 0;
    let fail = false;
    let subscription = node.createSubscription(
      msgString,
      'String_channel',
      options,
      (msg) => {
        msgCnt++;
        if (msg.data != 'FilteredData') {
          fail = true;
        }
      }
    );

    assert.ok(subscription.isContentFilteringEnabled());

    let publisher1 = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'String',
      "'FilteredData'",
    ]);

    let publisher2 = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'String',
      "'Data'",
    ]);

    setTimeout(() => {
      publisher1.kill('SIGINT');
      publisher2.kill('SIGINT');
      assert.ok(msgCnt && !fail);
      done();
    }, 1000);

    rclnodejs.spin(node);
  });

  it('single parameter', function (done) {
    if (!isContentFilteringSupported()) {
      this.skip();
    }

    let node = rclnodejs.createNode('string_subscription');
    let msgString = 'std_msgs/msg/String';
    let options = rclnodejs.Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = %0',
      parameters: ["'FilteredData'"],
    };

    let msgCnt = 0;
    let fail = false;
    let subscription = node.createSubscription(
      msgString,
      'String_channel',
      options,
      (msg) => {
        msgCnt++;
        if (msg.data != 'FilteredData') {
          fail = true;
        }
      }
    );

    assert.ok(subscription.isContentFilteringEnabled());

    let publisher1 = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'String',
      "'FilteredData'",
    ]);

    let publisher2 = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'String',
      "'Data'",
    ]);

    setTimeout(() => {
      publisher1.kill('SIGINT');
      publisher2.kill('SIGINT');
      assert.ok(msgCnt && !fail);
      done();
    }, 1000);

    rclnodejs.spin(node);
  });

  it('multiple parameters', function (done) {
    if (!isContentFilteringSupported()) {
      this.skip();
    }

    let node = rclnodejs.createNode('int32_subscription');
    let msgString = 'std_msgs/msg/Int32';
    let options = rclnodejs.Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data >= %0 AND data <= %1',
      parameters: [5, 10],
    };

    let msgCnt = 0;
    let fail = false;
    let subscription = node.createSubscription(
      msgString,
      'Int32_channel',
      options,
      (msg) => {
        msgCnt++;
        if (msg.data === 0) {
          fail = true;
        }
      }
    );

    assert.ok(subscription.isContentFilteringEnabled());

    let publisher1 = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'Int32',
      '0',
    ]);

    let publisher2 = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'Int32',
      '7',
    ]);

    setTimeout(() => {
      publisher1.kill('SIGINT');
      publisher2.kill('SIGINT');
      assert.ok(msgCnt && !fail);
      done();
    }, 1000);

    rclnodejs.spin(node);
  });

  it('no content-filter', function (done) {
    if (!isContentFilteringSupported()) {
      this.skip();
    }

    let node = rclnodejs.createNode('string_subscription');
    let msgString = 'std_msgs/msg/String';

    let msgCnt = 0;
    let subscription = node.createSubscription(
      msgString,
      'String_channel',
      (msg) => {
        msgCnt++;
      }
    );

    assert.ok(!subscription.isContentFilteringEnabled());

    let publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'String',
      "'Data'",
    ]);

    setTimeout(() => {
      publisher.kill('SIGINT');
      assert.ok(msgCnt > 0);
      done();
    }, 1000);

    rclnodejs.spin(node);
  });

  it('bad expression', function (done) {
    if (!isContentFilteringSupported()) {
      this.skip();
    }

    let node = rclnodejs.createNode('string_subscription');
    let msgString = 'std_msgs/msg/String';
    let options = rclnodejs.Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'this will fail',
    };

    let subscription;
    try {
      subscription = subscription = node.createSubscription(
        msgString,
        'String_channel',
        options,
        (msg) => {}
      );
    } catch (e) {}

    assert.ok(!subscription || !subscription.isContentFilteringEnabled());
    done();
  });
});
