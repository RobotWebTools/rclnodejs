'use strict';

const childProcess = require('child_process');
const assert = require('assert');
const rclnodejs = require('../index.js');
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
    RMWUtils.getRMWName() != RMWUtils.RMWNames.CYCLONEDDS
  );
}

function createAndRunPublisher(node, typeclass, topic, msgValue, interval=PUBLISHER_INTERVAL) {
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

  before(function() {
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
    this.intervals.forEach(interval => clearInterval(interval));
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

  it('no parameters', async function() {
    const typeclass = 'std_msgs/msg/String';
    const publisherTimer1 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        'FilteredData'
      );
    this.intervals.push(publisherTimer1);
  
    const publisherTimer2 = 
      createAndRunPublisher(
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
        if (msg.data != 'FilteredData')  fail = true;
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

  it('single parameter', async function() {
    const typeclass = 'std_msgs/msg/String';
    const publisherTimer1 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        'FilteredData'
      );
    this.intervals.push(publisherTimer1);
  
    const publisherTimer2 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        'Data',
        200);
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

  it('multiple parameters', async function() {
    const typeclass = 'std_msgs/msg/Int32';
    const publisherTimer1 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        0
      );
    this.intervals.push(publisherTimer1);
  
    const publisherTimer2 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        7,
        200);
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

  it('setContentFilter', async function() {
    const typeclass = 'std_msgs/msg/Int32';
    const publisherTimer1 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        0
      );
    this.intervals.push(publisherTimer1);
  
    const publisherTimer2 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        5,
        200);
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

    const p1 = new Promise((resolve) => 
      setTimeout(() => {
        let msgCnt = msgCnt0 + msgCnt5;
        const contentFilter5 = {
          expression: 'data = 5',
        };
        subscription.setContentFilter(contentFilter5);
        resolve(msgCnt);
      }, SUBSCRIBER_WAIT_TIME)
    );
    let result = await p1;
    assert.strictEqual(result, 0);

    const p2 = new Promise((resolve) => 
      setTimeout(() => {
        resolve(!fail && msgCnt5 && !msgCnt0);
      }, SUBSCRIBER_WAIT_TIME)
    );
    result = await p2;
    assert.ok(result);
  });

  it('set undefined content filter', async function() {
    const typeclass = 'std_msgs/msg/Int32';
    const publisherTimer1 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        0
      );
    this.intervals.push(publisherTimer1);
  
    const publisherTimer2 = 
      createAndRunPublisher(
        this.publisherNode,
        typeclass,
        TOPIC,
        5,
        200);
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

    const p1 = new Promise((resolve) => 
      setTimeout(() => {
        const result = !msgCnt0 && msgCnt5 && !fail;
        subscription.setContentFilter();
        resolve(result);
      }, SUBSCRIBER_WAIT_TIME)
    );
    let result = await p1;
    assert.ok(result);
    assert.ok(!subscription.hasContentFilter());

    const p2 = new Promise((resolve) => 
      setTimeout(() => {
        resolve(msgCnt0 && msgCnt5 && !fail);
      }, SUBSCRIBER_WAIT_TIME)
    );
    result = await p2;
    assert.ok(result);
  });

  it('clearContentFilter', function(done) {
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

  it('multiple clearContentFilter', function(done) {
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
