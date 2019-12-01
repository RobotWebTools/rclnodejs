/// <reference path="../lib/index.d.ts" />
import * as rclnodejs from 'rclnodejs';

const NODE_NAME = "test_node";
const TYPE_CLASS = "";
const TOPIC = "topic";


// ---- rclnodejs -----
// $ExpectType Promise<void>
rclnodejs.init();

// $ExpectType Promise<void>
rclnodejs.init(rclnodejs.Context.defaultContext());

// $ExpectType boolean
rclnodejs.isShutdown();

// $ExpectType void
rclnodejs.shutdown();


// ---- Node -----
// $ExpectType Node
const node = rclnodejs.createNode(NODE_NAME);

// $ExpectType string
node.name();

// $ExpectType string
node.namespace();

// $ExpectType void
rclnodejs.spin(node);

// $ExpectType void
node.destroy();

// $ExpectType NamesAndTypesQueryResult[]
node.getPublisherNamesAndTypesByNode(NODE_NAME);

// $ExpectType NamesAndTypesQueryResult[]
node.getServiceNamesAndTypes();

// $ExpectType NamesAndTypesQueryResult[]
node.getServiceNamesAndTypesByNode(NODE_NAME);

// $ExpectType NamesAndTypesQueryResult[]
node.getSubscriptionNamesAndTypesByNode(NODE_NAME);

// $ExpectType NamesAndTypesQueryResult[]
node.getTopicNamesAndTypes();


// ---- Timer ----
// ExpectType rclnodejs.TimerRequestCallback
const timerCallback = () => { };

// $ExpectType Timer
const timer = node.createTimer(100, timerCallback);

// $ExpectType number
timer.period;

// $ExpectType boolean
timer.isReady();

// $ExpectType number
timer.timeSinceLastCall();

// $ExpectType number
timer.timeUntilNextCall();

// $ExpectType boolean
timer.isCanceled();

// $ExpectType void
timer.cancel();


// ---- Publisher ----
// $ExpectType Publisher
const publisher = node.createPublisher(TYPE_CLASS, TOPIC);

// $ExpectType object
publisher.options;

// $ExpectType QoS
publisher.qos;

// $ExpectType string
publisher.topic;

// $ExpectType TypeClass
publisher.typeClass;

// $ExpectType boolean
publisher.typedArrayEnabled;

// $ExpectType void
publisher.publish("");
