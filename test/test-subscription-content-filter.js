import childProcess from 'child_process';
import assert from 'assert';
import { setTimeout as delay } from 'node:timers/promises';
import rclnodejs from '../index.js';
const DistroUtils = rclnodejs.DistroUtils;

const RMWUtils = rclnodejs.RMWUtils;
const Node = rclnodejs.Node;
const DEFAULT_NODE_OPTIONS = Node.getDefaultOptions();
const TOPIC = 'test';
const PUBLISHER_INTERVAL = 200;
const SUBSCRIBER_WAIT_TIME = 1000;

function isContentFilteringSupported() {
  return (
    DistroUtils.getDistroId() >= DistroUtils.getDistroId('humble') &&
    RMWUtils.getRMWName() != RMWUtils.RMWNames.CYCLONEDDS &&
    RMWUtils.getRMWName() != RMWUtils.RMWNames.ZENOH
  );
}

function createAndRunPublisher(
  node,
  typeclass,
  topic,
  msgValue,
  interval = PUBLISHER_INTERVAL
) {
  const publisher = node.createPublisher(typeclass, topic);
  const msg = rclnodejs.createMessage(typeclass);
  msg.data = msgValue;
  const timer = setInterval(() => {
    publisher.publish(msg);
  }, interval);
  return timer;
}

describe('subscription content-filtering', function () {
  let publisherNode;
  let subscriberNode;
  let intervals;

  this.timeout(30 * 1000);

  before(function () {
    if (!isContentFilteringSupported()) {
      this.skip();
    }
  });

  beforeEach(async function () {
    await rclnodejs.init();
    this.publisherNode = new Node('ctf_test_publisher_node');
    this.subscriberNode = new Node('ctf_test_subscriber_node');
    this.intervals = [];
  });

  afterEach(function () {
    this.intervals.forEach((interval) => clearInterval(interval));
    this.publisherNode.destroy();
    this.subscriberNode.destroy();
    rclnodejs.shutdown();
  });

  it('isContentFilteringEnabled', function (done) {
    const typeclass = 'std_msgs/msg/Int16';
    const options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = 16',
    };
    let subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {}
    );

    assert.ok(subscription.hasContentFilter());

    this.subscriberNode.destroySubscription(subscription);
    subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      (msg) => {}
    );
    assert.ok(!subscription.hasContentFilter());

    done();
  });

  it('no parameters', async function () {
    const typeclass = 'std_msgs/msg/String';
    const publisherTimer1 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      'FilteredData'
    );
    this.intervals.push(publisherTimer1);

    const publisherTimer2 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      'Data'
    );
    this.intervals.push(publisherTimer2);

    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: "data = 'FilteredData'",
    };

    let msgCnt = 0;
    let fail = false;
    let subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {
        msgCnt++;
        if (msg.data != 'FilteredData') fail = true;
      }
    );

    assert.ok(subscription.hasContentFilter());

    this.subscriberNode.spin();

    const p = new Promise((resolve) =>
      setTimeout(() => {
        resolve(msgCnt && !fail);
      }, SUBSCRIBER_WAIT_TIME)
    );
    let result = await p;

    assert.ok(result);
  });

  it('single parameter', async function () {
    const typeclass = 'std_msgs/msg/String';
    const publisherTimer1 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      'FilteredData'
    );
    this.intervals.push(publisherTimer1);

    const publisherTimer2 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      'Data',
      200
    );
    this.intervals.push(publisherTimer2);

    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = %0',
      parameters: ["'FilteredData'"],
    };

    let msgCnt = 0;
    let fail = false;
    let subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {
        msgCnt++;
        if (msg.data != 'FilteredData') fail = true;
      }
    );

    assert.ok(subscription.hasContentFilter());

    this.subscriberNode.spin();

    const p = new Promise((resolve) =>
      setTimeout(() => {
        resolve(msgCnt && !fail);
      }, 1000)
    );
    let result = await p;

    assert.ok(result);
  });

  it('multiple parameters', async function () {
    const typeclass = 'std_msgs/msg/Int32';
    const publisherTimer1 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      0
    );
    this.intervals.push(publisherTimer1);

    const publisherTimer2 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      7,
      200
    );
    this.intervals.push(publisherTimer2);

    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data >= %0 AND data <= %1',
      parameters: [5, 10],
    };

    let msgCnt = 0;
    let fail = false;
    const subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {
        msgCnt++;
        if (msg.data === 0) fail = true;
      }
    );

    assert.ok(subscription.hasContentFilter());

    this.subscriberNode.spin();

    const p = new Promise((resolve) =>
      setTimeout(() => {
        resolve(msgCnt && !fail);
      }, 1000)
    );
    let result = await p;

    assert.ok(result);
  });

  it('setContentFilter', async function () {
    const typeclass = 'std_msgs/msg/Int32';
    const publisherTimer1 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      0
    );
    this.intervals.push(publisherTimer1);

    const publisherTimer2 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      5,
      200
    );
    this.intervals.push(publisherTimer2);

    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = %0',
      parameters: [3],
    };

    let msgCnt0 = 0;
    let msgCnt5 = 0;
    let fail = false;
    const subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {
        switch (msg.data) {
          case 0:
            msgCnt0++;
            break;
          case 5:
            msgCnt5++;
            break;
          default:
            fail = true;
        }
      }
    );

    assert.ok(subscription.hasContentFilter());

    this.subscriberNode.spin();

    await delay(SUBSCRIBER_WAIT_TIME);
    assert.strictEqual(msgCnt0 + msgCnt5, 0);
    const contentFilter5 = { expression: 'data = 5' };
    assert.strictEqual(subscription.setContentFilter(contentFilter5), true);
    assert.strictEqual(this.subscriberNode.spinning, true);

    await delay(SUBSCRIBER_WAIT_TIME);
    assert.ok(!fail && msgCnt5 && !msgCnt0);
  });

  it('set undefined content filter', async function () {
    const typeclass = 'std_msgs/msg/Int32';
    const publisherTimer1 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      0
    );
    this.intervals.push(publisherTimer1);

    const publisherTimer2 = createAndRunPublisher(
      this.publisherNode,
      typeclass,
      TOPIC,
      5,
      200
    );
    this.intervals.push(publisherTimer2);

    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = %0',
      parameters: [5],
    };

    let msgCnt0 = 0;
    let msgCnt5 = 0;
    let fail = false;
    const subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {
        switch (msg.data) {
          case 0:
            msgCnt0++;
            break;
          case 5:
            msgCnt5++;
            break;
          default:
            fail = true;
        }
      }
    );

    assert.ok(subscription.hasContentFilter());

    this.subscriberNode.spin();

    await delay(SUBSCRIBER_WAIT_TIME);
    assert.ok(!msgCnt0 && msgCnt5 && !fail);
    assert.strictEqual(subscription.setContentFilter(), true);
    assert.ok(!subscription.hasContentFilter());
    assert.strictEqual(this.subscriberNode.spinning, true);

    await delay(SUBSCRIBER_WAIT_TIME);
    assert.ok(msgCnt0 && msgCnt5 && !fail);
  });

  it('updates and clears a filter during an infinite spin', function (done) {
    this.timeout(15000);
    const moduleUrl = new URL('../index.js', import.meta.url).href;
    const script = `
      import assert from 'node:assert/strict';
      import { setTimeout as delay } from 'node:timers/promises';
      import rclnodejs from ${JSON.stringify(moduleUrl)};

      await rclnodejs.init();
      try {
        const receiver = rclnodejs.createNode('filter_update_receiver');
        const sender = rclnodejs.createNode('filter_update_sender');
        const type = 'std_msgs/msg/Int32';
        const topic = 'filter_update_while_spinning';
        const filter = { expression: 'data = %0', parameters: ['30'] };
        const publisher = sender.createPublisher(type, topic);
        let complete;
        const delivered = new Promise((resolve) => { complete = resolve; });
        const subscription = receiver.createSubscription(
          type, topic, { contentFilter: filter }, (message) => {
            if (message.data === 42) complete();
          }
        );

        receiver.spin(-1);
        await delay(100);
        assert.equal(subscription.clearContentFilter(), true);
        assert.equal(subscription.hasContentFilter(), false);

        for (let round = 0; round < 64; round++) {
          await delay(2);
          assert.equal(subscription.setContentFilter(filter), true);
          assert.equal(subscription.hasContentFilter(), true);
          publisher.publish({ data: 30 });
          await delay(2);
          assert.equal(subscription.clearContentFilter(), true);
          assert.equal(subscription.hasContentFilter(), false);
          publisher.publish({ data: 5 });
        }

        assert.equal(receiver.spinning, true);
        const deadline = Date.now() + 5000;
        while (
          (!subscription.publisherCount || !publisher.subscriptionCount) &&
          Date.now() < deadline
        ) {
          await delay(20);
        }
        assert.ok(subscription.publisherCount > 0);
        assert.ok(publisher.subscriptionCount > 0);
        publisher.publish({ data: 42 });
        await delivered;
      } finally {
        rclnodejs.shutdown();
      }
    `;

    childProcess.execFile(
      process.execPath,
      ['--input-type=module', '--eval', script],
      { timeout: 10000, killSignal: 'SIGKILL' },
      done
    );
  });

  it('clearContentFilter', function (done) {
    const typeclass = 'std_msgs/msg/Int32';
    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = %0',
      parameters: [5],
    };

    const subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {}
    );

    assert.ok(subscription.hasContentFilter());
    assert.ok(subscription.clearContentFilter());
    assert.ok(!subscription.hasContentFilter());

    done();
  });

  it('getContentFilter', function (done) {
    const typeclass = 'std_msgs/msg/Int32';
    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = %0',
      parameters: [5],
    };

    const subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {}
    );

    assert.ok(subscription.hasContentFilter());
    const filter = subscription.getContentFilter();
    assert.strictEqual(filter.expression, 'data = %0');
    assert.strictEqual(filter.parameters.length, 1);
    assert.strictEqual(filter.parameters[0], '5');

    done();
  });

  it('multiple clearContentFilter', function (done) {
    const typeclass = 'std_msgs/msg/Int32';
    let options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'data = %0',
      parameters: [5],
    };

    const subscription = this.subscriberNode.createSubscription(
      typeclass,
      TOPIC,
      options,
      (msg) => {}
    );

    assert.ok(subscription.hasContentFilter());
    assert.ok(subscription.clearContentFilter());
    assert.ok(!subscription.hasContentFilter());
    assert.ok(subscription.clearContentFilter());
    assert.ok(subscription.clearContentFilter());
    assert.ok(!subscription.hasContentFilter());

    done();
  });

  it('bad expression', function (done) {
    const typeclass = 'std_msgs/msg/Int16';
    const options = Node.getDefaultOptions();
    options.contentFilter = {
      expression: 'this will fail',
    };

    let subscription;
    try {
      this.subscriberNode.createSubscription(
        typeclass,
        TOPIC,
        options,
        (msg) => {}
      );
    } catch (err) {}

    assert.ok(!subscription || !subscription.hasContentFilter());

    done();
  });
});

describe('subscription isContentFilterSupported', function () {
  this.timeout(30 * 1000);

  beforeEach(async function () {
    await rclnodejs.init();
    this.node = new Node('cft_support_test_node');
  });

  afterEach(function () {
    this.node.destroy();
    rclnodejs.shutdown();
  });

  it('isContentFilterSupported returns boolean matching RMW capability', function (done) {
    const typeclass = 'std_msgs/msg/Int16';
    const subscription = this.node.createSubscription(
      typeclass,
      TOPIC,
      (msg) => {}
    );

    const supported = subscription.isContentFilterSupported();
    assert.strictEqual(typeof supported, 'boolean');

    // isContentFilterSupported requires Lyrical or newer; on older distros it
    // returns false.
    const isLyricalOrNewer =
      DistroUtils.getDistroId() >= DistroUtils.DistroId.LYRICAL;
    const expectedSupported = isLyricalOrNewer && isContentFilteringSupported();
    assert.strictEqual(supported, expectedSupported);

    done();
  });
});
