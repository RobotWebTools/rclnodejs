/// <reference path='../../types/index.d.ts' />
import * as rclnodejs from 'rclnodejs';

const NODE_NAME = 'test_node';
const TYPE_CLASS = 'std_msgs/msg/String';
const TOPIC = 'topic';

// ---- rclnodejs -----
// $ExpectType Promise<void>
rclnodejs.init();

// $ExpectType Promise<void>
rclnodejs.init(rclnodejs.Context.defaultContext());

// $ExpectType boolean
rclnodejs.isShutdown();

// $ExpectType void
rclnodejs.shutdown();

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

// $ExpectType string
node.name();

// $ExpectType string
node.namespace();

// $ExpectType Logging
node.getLogger();

// $ExpectType Clock
node.getClock();

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

// $ExpectType string[]
node.getNodeNames();

// $ExpectType NodeNamesQueryResult[]
node.getNodeNamesAndNamespaces();

// $ExpectType number
node.countPublishers(TOPIC);

// $ExpectType number
node.countSubscribers(TOPIC);

// ---- Publisher ----
// $ExpectType Publisher
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
publisher.publish('');

// ---- Subscription ----
// $ExpectType Subscription
const subscription = node.createSubscription(
  TYPE_CLASS,
  TOPIC,
  {},
  (msg: rclnodejs.Message) => {}
);

// $ExpectType string
subscription.topic;

// ---- Service ----
// $ExpectType Service
const service = node.createService(
  TYPE_CLASS,
  'abc',
  {},
  (request: rclnodejs.Message, response: rclnodejs.ServiceResponse) => {}
);

// $ExpectType string
service.serviceName;

// $ExpectType object
service.options;

// ---- Client ----
// $ExpectType Client
const client = node.createClient(TYPE_CLASS, 'abc');

// $ExpectType string
client.serviceName;

// $ExpectType boolean
client.isServiceServerAvailable();

// $ExpectType Promise<boolean>
client.waitForService();

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

// $ExpectType Duration | Time
time1.sub(duration1);

// $ExpectType Duration | Time
time1.sub(time2);

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

// FOXY-ONLY, example_interfaces introduced with foxy release
// ---- Int8Array ----
const i8arr = rclnodejs.require('example_interfaces.msg.Int8MultiArray') as rclnodejs.example_interfaces.msg.Int8MultiArray;
// $ExpectType number[] | Int8Array
i8arr.data;

// ---- Uint8Array ----
const u8arr = rclnodejs.require('example_interfaces.msg.UInt8MultiArray') as rclnodejs.example_interfaces.msg.UInt8MultiArray;
// $ExpectType number[] | Uint8Array
u8arr.data;

// ---- Int16Array ----
const i16arr = rclnodejs.require('example_interfaces.msg.Int16MultiArray') as rclnodejs.example_interfaces.msg.Int16MultiArray;
// $ExpectType number[] | Int16Array
i16arr.data;

// ---- Uint16Array ----
const u16arr = rclnodejs.require('example_interfaces.msg.UInt16MultiArray') as rclnodejs.example_interfaces.msg.UInt16MultiArray;
// $ExpectType number[] | Uint16Array
u16arr.data;

// ---- Int32Array ----
const i32arr = rclnodejs.require('example_interfaces.msg.Int32MultiArray') as rclnodejs.example_interfaces.msg.Int32MultiArray;
// $ExpectType number[] | Int32Array
i32arr.data;

// ---- Uint16Array ----
const u32arr = rclnodejs.require('example_interfaces.msg.UInt32MultiArray') as rclnodejs.example_interfaces.msg.UInt32MultiArray;
// $ExpectType number[] | Uint32Array
u32arr.data;

// ---- Float32Array ----
const f32arr = rclnodejs.require('example_interfaces.msg.Float32MultiArray') as rclnodejs.example_interfaces.msg.Float32MultiArray;
// $ExpectType number[] | Float32Array
f32arr.data;

// ---- Float64Array ----
const f64arr = rclnodejs.require('example_interfaces.msg.Float64MultiArray') as rclnodejs.example_interfaces.msg.Float64MultiArray;
// $ExpectType number[] | Float64Array
f64arr.data;


// $ExpectType FibonacciConstructor
const Fibonacci = rclnodejs.require('rclnodejs_test_msgs/action/Fibonacci');

// ---- ActionClient -----
// $ExpectType ActionClient<"rclnodejs_test_msgs/action/Fibonacci">
const actionClient = new rclnodejs.ActionClient(
  node,
  'rclnodejs_test_msgs/action/Fibonacci',
  'fibonnaci'
);

// $ExpectType boolean
client.isServiceServerAvailable();

// $ExpectType Promise<boolean>
actionClient.waitForServer();

// $ExpectType Promise<ClientGoalHandle<"rclnodejs_test_msgs/action/Fibonacci">>
const goalHandlePromise = actionClient.sendGoal(new Fibonacci.Goal());

goalHandlePromise.then(goalHandle => {
  // $ExpectType boolean
  goalHandle.accepted;

  // $ExpectType UUID
  goalHandle.goalId;

  // $ExpectType Time
  goalHandle.stamp;

  // $ExpectType string
  goalHandle.status;

  // $ExpectType Promise<CancelGoal_Response>
  goalHandle.cancelGoal();

  // $ExpectType Promise<Fibonacci_Result>
  goalHandle.getResult();
});

// ---- ActionServer -----
// $ExpectType ActionServer<"rclnodejs_test_msgs/action/Fibonacci">
const actionServer = new rclnodejs.ActionServer(
  node,
  'rclnodejs_test_msgs/action/Fibonacci',
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

function executeCallback(
  goalHandle: rclnodejs.ServerGoalHandle<'rclnodejs_test_msgs/action/Fibonacci'>
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


