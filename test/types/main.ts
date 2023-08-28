/// <reference path='../../types/index.d.ts' />
import * as rclnodejs from 'rclnodejs';

const NODE_NAME = 'test_node';
const LIFECYCLE_NODE_NAME = 'lifecycle_test_node';
const TYPE_CLASS = 'std_msgs/msg/String';
const TOPIC = 'topic';
const MSG = rclnodejs.createMessageObject(TYPE_CLASS);
MSG.data = '';

// ---- rclnodejs -----
// $ExpectType Promise<void>
rclnodejs.init();

// $ExpectType Promise<void>
rclnodejs.init(rclnodejs.Context.defaultContext());

// $ExpectType boolean
rclnodejs.isShutdown();

// $ExpectType void
rclnodejs.shutdown();

// ---- DistroUtil ----

// $ExpectType DistroId
rclnodejs.DistroUtils.getDistroId();

// $ExpectType DistroId
rclnodejs.DistroUtils.getDistroId('foxy');

// $ExpectType string | undefined
rclnodejs.DistroUtils.getDistroName();

// $ExpectType string | undefined
rclnodejs.DistroUtils.getDistroName(2105);

// ---- Context -----
// $ExpectType Context
const context = rclnodejs.Context.defaultContext();

// ---- NodeOptions ----
// $ExpectType NodeOptions
const nodeOptions = new rclnodejs.NodeOptions();

// $ExpectType boolean
nodeOptions.startParameterServices;

// $ExpectType boolean
nodeOptions.automaticallyDeclareParametersFromOverrides;

// $ExpectType Parameter[]
nodeOptions.parameterOverrides;

// ---- Node -----
// $ExpectType Node
const node = rclnodejs.createNode(NODE_NAME);

// $ExpectType Node
const node1 = new rclnodejs.Node(NODE_NAME + '1');

// $ExpectType string
node.name();

// $ExpectType string
node.namespace();

// $ExpectType Logging
node.getLogger();

// $ExpectType Clock
node.getClock();

// $ExpectType void
node.spin();

// $ExpectType void
node.spinOnce();

// $ExpectType boolean
node.spinning;

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

// $ExpectType string[]
node.getNodeNames();

// $ExpectType NodeNamesQueryResult[]
node.getNodeNamesAndNamespaces();

// $ExpectType number
node.countPublishers(TOPIC);

// $ExpectType number
node.countSubscribers(TOPIC);

// $ExpectType Options<string | QoS>
rclnodejs.Node.getDefaultOptions();

// ---- LifecycleNode ----
// $ExpectType LifecycleNode
const lifecycleNode = rclnodejs.createLifecycleNode(LIFECYCLE_NODE_NAME);

// $ExpectType LifecycleNode
const lifecycleNode1 = rclnodejs.createLifecycleNode(
  LIFECYCLE_NODE_NAME + '1',
  undefined,
  undefined,
  undefined,
  true
);

// $ExpectType LifecycleNode
const lifecycleNode2 = new rclnodejs.lifecycle.LifecycleNode(
  LIFECYCLE_NODE_NAME
);

// $ExpectType LifecycleNode
const lifecycleNode3 = new rclnodejs.lifecycle.LifecycleNode(
  LIFECYCLE_NODE_NAME + '3',
  undefined,
  undefined,
  undefined,
  true
);

// $ExpectType State
lifecycleNode.currentState;

// $ExpectType State[]
lifecycleNode.availableStates;

// $ExpectType TransitionDescription[]
lifecycleNode.transitions;

// $ExpectType TransitionDescription[]
lifecycleNode.availableTransitions;

//// $ExpectType TransitionCallback
// const lifecycleCB: TransitionCallback = (prevState: State) => CallbackReturnCode.SUCCESS;

// $ExpectType CallbackReturnValue
const ReturnValue = new rclnodejs.lifecycle.CallbackReturnValue();

// $ExpectType State
lifecycleNode.configure(ReturnValue);

// $ExpectType State
lifecycleNode.activate();

// $ExpectType State
lifecycleNode.deactivate();

// $ExpectType State
lifecycleNode.cleanup();

// $ExpectType State
lifecycleNode.shutdown();

// ---- Publisher ----
// $ExpectType Publisher<"std_msgs/msg/String">
const publisher = node.createPublisher(TYPE_CLASS, TOPIC);

// $ExpectType object
publisher.options;

// $ExpectType QoS
publisher.qos;

// $ExpectType string
publisher.topic;

// $ExpectType TypeClass<TypeClassName>
publisher.typeClass;

// $ExpectType boolean
publisher.typedArrayEnabled;

// $ExpectType void
publisher.publish(MSG);

// $ExpectType void
publisher.publish(Buffer.from('Hello ROS World'));

// $ExpectType void
node.destroyPublisher(publisher);

// $ExpectType boolean
publisher.isDestroyed();

// ---- LifecyclePublisher ----
// $ExpectType LifecyclePublisher<"std_msgs/msg/String">
const lifecyclePublisher = lifecycleNode.createLifecyclePublisher(
  TYPE_CLASS,
  TOPIC
);

// $ExpectType boolean
lifecyclePublisher.isActivated();

// ---- Subscription ----
// $ExpectType Subscription
let subscription = node.createSubscription(TYPE_CLASS, TOPIC, (msg) => {});

// $ExpectType Subscription
subscription = node.createSubscription(TYPE_CLASS, TOPIC, {}, (msg) => {});

const contentFilter: rclnodejs.SubscriptionContentFilter = {
  expression: 'data < %0',
  parameters: [5],
};

// $ExpectType Subscription
subscription = node.createSubscription(
  TYPE_CLASS,
  TOPIC,
  { contentFilter },
  (msg) => {}
);

// $ExpectType string
subscription.topic;

// $ExpectType boolean
subscription.isDestroyed();

subscription.setContentFilter(contentFilter);

// $ExpectType boolean
subscription.clearContentFilter();

// $ExpectType boolean
subscription.hasContentFilter();

// ---- Service ----
// $ExpectType AddTwoIntsConstructor
const service = node.createService(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints',
  (request, response) => {}
);

// $ExpectType string
service.serviceName;

// $ExpectType object
service.options;

service.configureIntrospection(
  node.getClock(),
  rclnodejs.Node.getDefaultOptions() as rclnodejs.QoS,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);

// $ExpectType boolean
service.isDestroyed();

// ---- Client ----
// $ExpectType Client<"example_interfaces/srv/AddTwoInts">
const client = node.createClient(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints'
);

// $ExpectType string
client.serviceName;

// $ExpectType boolean
client.isServiceServerAvailable();

// $ExpectType Promise<boolean>
client.waitForService();

client.configureIntrospection(
  node.getClock(),
  rclnodejs.Node.getDefaultOptions() as rclnodejs.QoS,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);

// $ExpectType boolean
client.isDestroyed();

// ---- Timer ----
// ExpectType rclnodejs.TimerRequestCallback
const timerCallback = () => {};

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

(async () => {
  // ---- Rate ----
  // $ExpectType Rate
  const rate = await node.createRate(1);

  // $ExpectType number
  rate.frequency;

  // $ExpectType boolean
  rate.isCanceled();

  // $ExpectType Promise<void>
  rate.sleep();

  // $ExpectType void
  rate.cancel();
})();

// ---- Duration ----
// $ExpectType Duration
const duration1: rclnodejs.Duration = new rclnodejs.Duration();

const duration2: rclnodejs.Duration = new rclnodejs.Duration(100, '1000');

// $ExpectType string | number
duration1.nanoseconds;

// $ExpectType boolean
duration1.eq(duration2);

// $ExpectType boolean
duration1.ne(duration2);

// $ExpectType boolean
duration1.lt(duration2);

// $ExpectType boolean
duration1.lte(duration2);

// $ExpectType boolean
duration1.gt(duration2);

// $ExpectType boolean
duration1.gte(duration2);

// ---- Time ----
// $ExpectType Time
const time1 = new rclnodejs.Time(100, 100);

// $ExpectType Time
const time2 = rclnodejs.Time.fromMsg({ sec: 0, nanosec: 0 });

// $ExpectType Time
const time3 = rclnodejs.Time.fromMsg(
  { sec: 0, nanosec: 0 },
  rclnodejs.ClockType.ROS_TIME
);

// $ExpectType ClockType
time1.clockType;

// $ExpectType string | number
time1.nanoseconds;

// $ExpectType { seconds: number; nanoseconds: number; }
time1.secondsAndNanoseconds;

// $ExpectType Time
time1.add(duration1);

// TODO: wayne - readd the 2 failing expect cases for Time|Duration transposed failure

// $ExpectType boolean
time1.eq(time2);

// $ExpectType boolean
time1.ne(time2);

// $ExpectType boolean
time1.lt(time2);

// $ExpectType boolean
time1.lte(time2);

// $ExpectType boolean
time1.gt(time2);

// $ExpectType boolean
time1.gte(time2);

// $ExpectType Time
time3.toMsg();

// ---- Clock -----
// $ExpectType Clock
const clock = new rclnodejs.Clock(rclnodejs.ClockType.SYSTEM_TIME);

// $ExpectType ClockType
clock.clockType;

// $ExpectType Time
clock.now();

// ---- ROS Clock -----

// ---- Logging -----
// $ExpectType Logging
const logger = rclnodejs.Logging.getLogger('test_logger');

// $ExpectType string
logger.name;

// $ExpectType LoggingSeverity
logger.loggerEffectiveLevel;

// $ExpectType void
logger.setLoggerLevel(rclnodejs.Logging.LoggingSeverity.INFO);

// $ExpectType boolean
logger.debug('test msg');

// $ExpectType boolean
logger.info('test msg');

// $ExpectType boolean
logger.warn('test msg');

// $ExpectType boolean
logger.error('test msg');

// $ExpectType boolean
logger.fatal('test msg');

// TODO - reinstate the disabled (commented out) test due to
// somewhat false negatives reported by multiple version of typescript
// and undefined rules for how union type ordering

// // FOXY-ONLY, example_interfaces introduced with foxy release
// // ---- Int8Array ----
// const i8arr = rclnodejs.require(
//   'example_interfaces.msg.Int8MultiArray'
// ) as rclnodejs.example_interfaces.msg.Int8MultiArray;
// // $ExpectType Int8Array | number[]
// i8arr.data;

// // ---- Uint8Array ----
// const u8arr = rclnodejs.require(
//   'example_interfaces.msg.UInt8MultiArray'
// ) as rclnodejs.example_interfaces.msg.UInt8MultiArray;
// // $ExpectType Uint8Array | number[]
// u8arr.data;

// // ---- Int16Array ----
// const i16arr = rclnodejs.require(
//   'example_interfaces.msg.Int16MultiArray'
// ) as rclnodejs.example_interfaces.msg.Int16MultiArray;
// // $ExpectType Int16Array | number[]
// i16arr.data;

// // ---- Uint16Array ----
// const u16arr = rclnodejs.require(
//   'example_interfaces.msg.UInt16MultiArray'
// ) as rclnodejs.example_interfaces.msg.UInt16MultiArray;
// // $ExpectType Uint16Array | number[]
// u16arr.data;

// // ---- Int32Array ----
// const i32arr = rclnodejs.require(
//   'example_interfaces.msg.Int32MultiArray'
// ) as rclnodejs.example_interfaces.msg.Int32MultiArray;
// // $ExpectType Int32Array | number[]
// i32arr.data;

// // ---- Uint16Array ----
// const u32arr = rclnodejs.require(
//   'example_interfaces.msg.UInt32MultiArray'
// ) as rclnodejs.example_interfaces.msg.UInt32MultiArray;
// // $ExpectType Uint32Array | number[]
// u32arr.data;

// // ---- Float32Array ----
// const f32arr = rclnodejs.require(
//   'example_interfaces.msg.Float32MultiArray'
// ) as rclnodejs.example_interfaces.msg.Float32MultiArray;
// // $ExpectType Float32Array | number[]
// f32arr.data;

// // ---- Float64Array ----
// const f64arr = rclnodejs.require(
//   'example_interfaces.msg.Float64MultiArray'
// ) as rclnodejs.example_interfaces.msg.Float64MultiArray;
// // $ExpectType Float64Array | number[]
// f64arr.data;

// $ExpectType FibonacciConstructor
const Fibonacci = rclnodejs.require('example_interfaces/action/Fibonacci');

// ---- ActionClient -----
// $ExpectType ActionClient<"example_interfaces/action/Fibonacci">
const actionClient = new rclnodejs.ActionClient(
  node,
  'example_interfaces/action/Fibonacci',
  'fibonnaci'
);

// $ExpectType boolean
client.isServiceServerAvailable();

// $ExpectType Promise<boolean>
actionClient.waitForServer();

// $ExpectType void
actionClient.destroy();

// $ExpectType Promise<ClientGoalHandle<"example_interfaces/action/Fibonacci">>
const goalHandlePromise = actionClient.sendGoal(new Fibonacci.Goal());

goalHandlePromise.then((goalHandle) => {
  // $ExpectType boolean
  goalHandle.accepted; // deprecated

  // $ExpectType UUID
  goalHandle.goalId;

  // $ExpectType Time
  goalHandle.stamp;

  // $ExpectType number
  goalHandle.status;

  // $ExpectType Promise<CancelGoal_Response>
  goalHandle.cancelGoal();

  // $ExpectType Promise<Fibonacci_Result>
  goalHandle.getResult();

  // $ExpectType boolean
  goalHandle.isAccepted();

  // $ExpectType boolean
  goalHandle.isExecuting();

  // $ExpectType boolean
  goalHandle.isSucceeded();

  // $ExpectType boolean
  goalHandle.isCanceling();

  // $ExpectType boolean
  goalHandle.isCanceled();

  // $ExpectType boolean
  goalHandle.isAborted();
});

// ---- ActionServer -----
// $ExpectType ActionServer<"example_interfaces/action/Fibonacci">
const actionServer = new rclnodejs.ActionServer(
  node,
  'example_interfaces/action/Fibonacci',
  'fibonnaci',
  executeCallback
);

// $ExpectType void
actionServer.registerHandleAcceptedCallback();

// $ExpectType void
actionServer.registerGoalCallback();

// $ExpectType void
actionServer.registerCancelCallback();

// $ExpectType void
actionServer.registerExecuteCallback(() => new Fibonacci.Result());

// $ExpectType void
actionServer.destroy();

function executeCallback(
  goalHandle: rclnodejs.ServerGoalHandle<'example_interfaces/action/Fibonacci'>
) {
  // $ExpectType UUID
  goalHandle.goalId;

  // $ExpectType boolean
  goalHandle.isActive;

  // $ExpectType boolean
  goalHandle.isCancelRequested;

  // $ExpectType Fibonacci_Goal
  goalHandle.request;

  // $ExpectType string
  goalHandle.status;

  // $ExpectType void
  goalHandle.abort();

  // $ExpectType void
  goalHandle.canceled();

  // $ExpectType void
  goalHandle.execute();

  // $ExpectType void
  goalHandle.publishFeedback(new Fibonacci.Feedback());

  // $ExpectType void
  goalHandle.succeed();

  return new Fibonacci.Result();
}

// ---- ActionUuid -----
// $ExpectType ActionUuid
const actionUuid = new rclnodejs.ActionUuid();

// $ExpectType ActionUuid
const actionUuid1 = rclnodejs.ActionUuid.random();

// $ExpectType ActionUuid
const actionUuid2 = rclnodejs.ActionUuid.fromBytes(new Uint8Array([21, 31]));

// $ExpectType string
actionUuid.toString();

// $ExpectType Uint8Array
actionUuid.bytes;

// $ExpectType UUID
actionUuid.toMessage();

// $ExpectType UUID
rclnodejs.ActionUuid.randomMessage();
