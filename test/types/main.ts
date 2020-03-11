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

// $ExpectType TypeClass
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

// ---- Rate ----
// $ExpectType Rate
const rate = node.createRate(1);

// $ExpectType number
rate.frequency;

// $ExpectType boolean
rate.isCanceled();

// $ExpectType Promise<void>
rate.sleep();

// $ExpectType void
rate.cancel();

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
const time2 = rclnodejs.Time.fromMsg({sec: 0, nanosec: 0});

// $ExpectType Time
const time3 =
  rclnodejs.Time.fromMsg({sec: 0, nanosec: 0}, rclnodejs.ClockType.ROS_TIME);

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
