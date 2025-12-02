/// <reference path='../../types/index.d.ts' />

import { expectType, expectAssignable } from 'tsd';
import * as rclnodejs from 'rclnodejs';
import { ChildProcess } from 'child_process';

const NODE_NAME = 'test_node';
const LIFECYCLE_NODE_NAME = 'lifecycle_test_node';
const TYPE_CLASS = 'std_msgs/msg/String';
const TOPIC = 'topic';
const SERVICE_NAME = 'service';
const MSG = rclnodejs.createMessageObject(TYPE_CLASS);
MSG.data = '';

// ---- rclnodejs -----
expectType<Promise<void>>(rclnodejs.init());
expectType<string | undefined>(rclnodejs.DistroUtils.getDistroName());
expectType<boolean>(rclnodejs.isShutdown());
expectType<void>(rclnodejs.shutdown());
expectType<void>(rclnodejs.removeSignalHandlers());
expectType<Promise<{ process: ChildProcess }>>(
  rclnodejs.ros2Run('package_name', 'executable_name', ['arg1', 'arg2'])
);
expectType<Promise<{ process: ChildProcess }>>(
  rclnodejs.ros2Launch('package_name', 'launch_file', ['arg1', 'arg2'])
);

// ---- DistroUtil ----
expectType<rclnodejs.DistroUtils.DistroId>(rclnodejs.DistroUtils.getDistroId());
expectType<rclnodejs.DistroUtils.DistroId>(
  rclnodejs.DistroUtils.getDistroId('foxy')
);
expectType<string | undefined>(rclnodejs.DistroUtils.getDistroName());
expectType<string | undefined>(rclnodejs.DistroUtils.getDistroName(2105));

// ---- Context -----
expectType<rclnodejs.Context>(rclnodejs.Context.defaultContext());
expectType<rclnodejs.Context>(new rclnodejs.Context(123n));
expectType<bigint>(rclnodejs.Context.defaultContext().domainId());

// ---- NodeOptions ----
const nodeOptions = new rclnodejs.NodeOptions();
expectType<rclnodejs.NodeOptions>(nodeOptions);
expectType<boolean>(nodeOptions.startParameterServices);
expectType<boolean>(nodeOptions.automaticallyDeclareParametersFromOverrides);
expectType<rclnodejs.Parameter[]>(nodeOptions.parameterOverrides);

// ---- Node -----
const node = rclnodejs.createNode(NODE_NAME);
expectType<rclnodejs.Node>(node);
expectType<rclnodejs.Node>(new rclnodejs.Node(NODE_NAME + '1'));
expectType<string>(node.name());
expectType<string>(node.namespace());
expectType<rclnodejs.Logging>(node.getLogger());
expectType<rclnodejs.Clock>(node.getClock());
expectType<void>(node.spin());
expectType<void>(node.spinOnce());
expectType<boolean>(node.spinning);
expectType<void>(node.destroy());
expectType<rclnodejs.NamesAndTypesQueryResult[]>(
  node.getPublisherNamesAndTypesByNode(NODE_NAME)
);
expectType<rclnodejs.NamesAndTypesQueryResult[]>(
  node.getServiceNamesAndTypes()
);
expectType<rclnodejs.NamesAndTypesQueryResult[]>(
  node.getServiceNamesAndTypesByNode(NODE_NAME)
);
expectType<rclnodejs.NamesAndTypesQueryResult[]>(
  node.getClientNamesAndTypesByNode(NODE_NAME)
);
expectType<rclnodejs.NamesAndTypesQueryResult[]>(
  node.getSubscriptionNamesAndTypesByNode(NODE_NAME)
);
expectType<rclnodejs.NamesAndTypesQueryResult[]>(node.getTopicNamesAndTypes());
expectType<string[]>(node.getNodeNames());
expectType<rclnodejs.NodeNamesQueryResult[]>(node.getNodeNamesAndNamespaces());
expectType<rclnodejs.NodeNamesQueryResultWithEnclaves[]>(
  node.getNodeNamesAndNamespacesWithEnclaves()
);
expectType<Array<object>>(node.getPublishersInfoByTopic('topic', false));
expectType<Array<object>>(node.getSubscriptionsInfoByTopic('topic', false));
expectType<number>(node.countPublishers(TOPIC));
expectType<number>(node.countSubscribers(TOPIC));
expectType<number>(node.countClients(SERVICE_NAME));
expectType<number>(node.countServices(SERVICE_NAME));
expectType<rclnodejs.Options<string | rclnodejs.QoS>>(
  rclnodejs.Node.getDefaultOptions()
);
expectType<string>(node.getFullyQualifiedName());
expectType<string>(node.getRMWImplementationIdentifier());
const nodeWithArgs = rclnodejs.createNode(
  NODE_NAME,
  'topic',
  rclnodejs.Context.defaultContext(),
  rclnodejs.NodeOptions.defaultOptions,
  ['--ros-args', '-r', '__ns:=/foo/bar'],
  false
);
expectType<rclnodejs.Node>(nodeWithArgs);
expectType<string>(node.resolveTopicName(TOPIC, true));
expectType<string>(node.resolveServiceName(SERVICE_NAME, true));

// ---- LifecycleNode ----
const lifecycleNode = rclnodejs.createLifecycleNode(LIFECYCLE_NODE_NAME);
expectType<rclnodejs.lifecycle.LifecycleNode>(lifecycleNode);

const lifecycleNode1 = rclnodejs.createLifecycleNode(
  LIFECYCLE_NODE_NAME + '1',
  undefined,
  undefined,
  undefined,
  true
);
expectType<rclnodejs.lifecycle.LifecycleNode>(lifecycleNode1);

const lifecycleNode2 = new rclnodejs.lifecycle.LifecycleNode(
  LIFECYCLE_NODE_NAME
);
expectType<rclnodejs.lifecycle.LifecycleNode>(lifecycleNode2);

const lifecycleNode3 = new rclnodejs.lifecycle.LifecycleNode(
  LIFECYCLE_NODE_NAME + '3',
  undefined,
  undefined,
  undefined,
  true
);
expectType<rclnodejs.lifecycle.LifecycleNode>(lifecycleNode3);

expectType<rclnodejs.lifecycle.State>(lifecycleNode.currentState);
expectType<rclnodejs.lifecycle.State[]>(lifecycleNode.availableStates);
expectType<rclnodejs.lifecycle.TransitionDescription[]>(
  lifecycleNode.transitions
);
expectType<rclnodejs.lifecycle.TransitionDescription[]>(
  lifecycleNode.availableTransitions
);

const ReturnValue = new rclnodejs.lifecycle.CallbackReturnValue();
expectType<rclnodejs.lifecycle.CallbackReturnValue>(ReturnValue);
expectType<rclnodejs.lifecycle.State>(lifecycleNode.configure(ReturnValue));
expectType<rclnodejs.lifecycle.State>(lifecycleNode.activate());
expectType<rclnodejs.lifecycle.State>(lifecycleNode.deactivate());
expectType<rclnodejs.lifecycle.State>(lifecycleNode.cleanup());
expectType<rclnodejs.lifecycle.State>(lifecycleNode.shutdown());
expectType<boolean>(lifecycleNode.isInitialized);
expectType<void>(lifecycleNode.print());

// ---- Publisher ----
const publisher = node.createPublisher(TYPE_CLASS, TOPIC);
expectType<rclnodejs.Publisher<'std_msgs/msg/String'>>(publisher);
expectType<object>(publisher.options);
expectType<rclnodejs.QoS>(publisher.qos);
expectType<string>(publisher.topic);
expectType<rclnodejs.TypeClass<rclnodejs.TypeClassName>>(publisher.typeClass);
expectType<boolean>(publisher.typedArrayEnabled);
expectType<void>(publisher.publish(MSG));
expectType<void>(publisher.publish(Buffer.from('Hello ROS World')));
expectType<void>(node.destroyPublisher(publisher));
expectType<boolean>(publisher.isDestroyed());
expectType<boolean>(publisher.waitForAllAcked(BigInt(1000)));
node.createPublisher(TYPE_CLASS, TOPIC, publisher.options, (event: object) => {
  const receivedEvent = event;
});
expectType<string>(publisher.loggerName);

// ---- LifecyclePublisher ----
const lifecyclePublisher = lifecycleNode.createLifecyclePublisher(
  TYPE_CLASS,
  TOPIC
);
expectType<rclnodejs.lifecycle.LifecyclePublisher<'std_msgs/msg/String'>>(
  lifecyclePublisher
);
expectType<boolean>(lifecyclePublisher.isActivated());

// ---- Subscription ----
let subscription = node.createSubscription(TYPE_CLASS, TOPIC, () => {});
expectType<rclnodejs.Subscription>(subscription);
expectType<rclnodejs.Subscription>(
  node.createSubscription(
    TYPE_CLASS,
    TOPIC,
    {},
    () => {},
    (event: object) => {
      const receivedEvent = event;
    }
  )
);

const contentFilter: rclnodejs.SubscriptionContentFilter = {
  expression: 'data < %0',
  parameters: [5],
};

subscription = node.createSubscription(
  TYPE_CLASS,
  TOPIC,
  { contentFilter },
  () => {}
);
expectType<rclnodejs.Subscription>(subscription);

subscription = node.createSubscription(
  TYPE_CLASS,
  TOPIC,
  { isRaw: false },
  (message: rclnodejs.std_msgs.msg.String) => {
    const receivedMessage = message;
  }
);

const rawMessageCallback = (message: Buffer) => {
  const receivedRawMessage = message;
};
expectType<rclnodejs.SubscriptionWithRawMessageCallback>(rawMessageCallback);

expectType<string>(subscription.topic);
expectType<boolean>(subscription.isDestroyed());
expectType<boolean>(subscription.setContentFilter(contentFilter));
expectType<boolean>(subscription.clearContentFilter());
expectType<boolean>(subscription.hasContentFilter());
expectType<string>(subscription.loggerName);

// ---- Service ----
const service = node.createService(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints',
  () => {}
);
expectType<rclnodejs.example_interfaces.srv.AddTwoIntsConstructor>(service);

expectType<string>(service.serviceName);
expectType<object>(service.options);
expectType<void>(
  service.configureIntrospection(
    node.getClock(),
    rclnodejs.Node.getDefaultOptions() as rclnodejs.QoS,
    rclnodejs.ServiceIntrospectionStates.CONTENTS
  )
);
expectType<boolean>(service.isDestroyed());
expectType<object>(service.getOptions());
expectType<string>(service.loggerName);

// ---- Client ----
const client = node.createClient(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints'
);
expectType<rclnodejs.Client<'example_interfaces/srv/AddTwoInts'>>(client);
expectType<string>(client.serviceName);
expectType<boolean>(client.isServiceServerAvailable());
expectType<Promise<boolean>>(client.waitForService());
expectType<void>(
  client.configureIntrospection(
    node.getClock(),
    rclnodejs.Node.getDefaultOptions() as rclnodejs.QoS,
    rclnodejs.ServiceIntrospectionStates.CONTENTS
  )
);
expectType<boolean>(client.isDestroyed());
expectType<string>(client.loggerName);

// ---- Timer ----
const timerCallback = () => {};
expectType<rclnodejs.TimerRequestCallback>(timerCallback);

const timer = node.createTimer(BigInt(100000), timerCallback);
expectType<rclnodejs.Timer>(timer);
expectType<bigint>(timer.period);
expectType<boolean>(timer.isReady());
expectType<bigint>(timer.timeSinceLastCall());
expectType<bigint>(timer.timeUntilNextCall());
expectType<boolean>(timer.isCanceled());
expectType<void>(timer.cancel());
expectType<void>(timer.changeTimerPeriod(BigInt(100000)));
expectType<bigint>(timer.timerPeriod());
expectType<object>(timer.callTimerWithInfo());

// ---- Rate ----
const rate = await node.createRate(1);
expectType<rclnodejs.Rate>(rate);
expectType<number>(rate.frequency);
expectType<boolean>(rate.isCanceled());
expectType<Promise<void>>(rate.sleep());
expectType<void>(rate.cancel());

// ---- Duration ----
const duration1 = new rclnodejs.Duration();
expectType<rclnodejs.Duration>(duration1);

const duration2 = new rclnodejs.Duration(BigInt(100), BigInt(1000));
expectType<rclnodejs.Duration>(duration2);
expectType<bigint>(duration1.nanoseconds);
expectType<boolean>(duration1.eq(duration2));
expectType<boolean>(duration1.ne(duration2));
expectType<boolean>(duration1.lt(duration2));
expectType<boolean>(duration1.lte(duration2));
expectType<boolean>(duration1.gt(duration2));
expectType<boolean>(duration1.gte(duration2));

// ---- Time ----
const time1 = new rclnodejs.Time(BigInt(100), BigInt(100));
expectType<rclnodejs.Time>(time1);

const time2 = rclnodejs.Time.fromMsg({ sec: 0, nanosec: 0 });
expectType<rclnodejs.Time>(time2);

const time3 = rclnodejs.Time.fromMsg(
  { sec: 0, nanosec: 0 },
  rclnodejs.ClockType.ROS_TIME
);
expectType<rclnodejs.Time>(time3);

expectType<rclnodejs.ClockType>(time1.clockType);
expectType<bigint>(time1.nanoseconds);
expectType<{ seconds: bigint; nanoseconds: bigint }>(
  time1.secondsAndNanoseconds
);
expectType<rclnodejs.Time>(time1.add(duration1));
expectType<boolean>(time1.eq(time2));
expectType<boolean>(time1.ne(time2));
expectType<boolean>(time1.lt(time2));
expectType<boolean>(time1.lte(time2));
expectType<boolean>(time1.gt(time2));
expectType<boolean>(time1.gte(time2));
expectType<rclnodejs.builtin_interfaces.msg.Time>(time3.toMsg());

// ---- Clock -----
const clock = new rclnodejs.Clock(rclnodejs.ClockType.SYSTEM_TIME);
expectType<rclnodejs.Clock>(clock);
expectType<rclnodejs.ClockType>(clock.clockType);
expectType<rclnodejs.Time>(clock.now());

// ---- Logging -----
const logger = rclnodejs.Logging.getLogger('test_logger');
expectType<rclnodejs.Logging>(logger);
expectType<string>(logger.name);
expectType<rclnodejs.Logging.LoggingSeverity>(logger.loggerEffectiveLevel);
expectType<void>(logger.setLoggerLevel(rclnodejs.Logging.LoggingSeverity.INFO));
expectType<boolean>(logger.debug('test msg'));
expectType<boolean>(logger.info('test msg'));
expectType<boolean>(logger.debug('test msg'));
expectType<boolean>(logger.warn('test msg'));
expectType<boolean>(logger.error('test msg'));
expectType<boolean>(logger.fatal('test msg'));
expectType<string>(rclnodejs.Logging.getLoggingDirectory());

// ---- ActionClient -----
const Fibonacci = rclnodejs.require('example_interfaces/action/Fibonacci');
expectType<rclnodejs.example_interfaces.action.FibonacciConstructor>(Fibonacci);

const actionClient = new rclnodejs.ActionClient(
  node,
  'example_interfaces/action/Fibonacci',
  'fibonnaci'
);
expectType<rclnodejs.ActionClient<'example_interfaces/action/Fibonacci'>>(
  actionClient
);
expectType<boolean>(client.isServiceServerAvailable());
expectType<Promise<boolean>>(actionClient.waitForServer());
expectType<void>(actionClient.destroy());

const goalHandlePromise = actionClient.sendGoal(new Fibonacci.Goal());
expectType<
  Promise<rclnodejs.ClientGoalHandle<'example_interfaces/action/Fibonacci'>>
>(goalHandlePromise);

goalHandlePromise.then((goalHandle) => {
  expectType<boolean>(goalHandle.accepted);
  expectType<rclnodejs.unique_identifier_msgs.msg.UUID>(goalHandle.goalId);
  expectType<rclnodejs.builtin_interfaces.msg.Time>(goalHandle.stamp);
  expectType<number>(goalHandle.status);
  expectType<Promise<rclnodejs.action_msgs.srv.CancelGoal_Response>>(
    goalHandle.cancelGoal()
  );
  expectType<Promise<rclnodejs.example_interfaces.action.Fibonacci_Result>>(
    goalHandle.getResult()
  );
  expectType<boolean>(goalHandle.isAccepted());
  expectType<boolean>(goalHandle.isExecuting());
  expectType<boolean>(goalHandle.isSucceeded());
  expectType<boolean>(goalHandle.isCanceling());
  expectType<boolean>(goalHandle.isCanceled());
  expectType<boolean>(goalHandle.isAborted());
});
expectType<object>(actionClient.getNumEntities());
expectType<void>(
  actionClient.configureIntrospection(
    node.getClock(),
    rclnodejs.Node.getDefaultOptions() as rclnodejs.QoS,
    rclnodejs.ServiceIntrospectionStates.CONTENTS
  )
);

// ---- ActionServer -----
const actionServer = new rclnodejs.ActionServer(
  node,
  'example_interfaces/action/Fibonacci',
  'fibonacci',
  executeCallback,
  goalCallback
);
expectType<rclnodejs.ActionServer<'example_interfaces/action/Fibonacci'>>(
  actionServer
);
expectType<void>(actionServer.registerHandleAcceptedCallback());
expectType<void>(actionServer.registerGoalCallback());
expectType<void>(actionServer.registerCancelCallback());
expectType<void>(
  actionServer.registerExecuteCallback(() => new Fibonacci.Result())
);
expectType<void>(actionServer.destroy());

function goalCallback(
  goal: rclnodejs.ActionGoal<'example_interfaces/action/Fibonacci'>
): rclnodejs.GoalResponse {
  expectType<number>(goal.order);
  return rclnodejs.GoalResponse.ACCEPT;
}

function executeCallback(
  goalHandle: rclnodejs.ServerGoalHandle<'example_interfaces/action/Fibonacci'>
) {
  expectType<rclnodejs.unique_identifier_msgs.msg.UUID>(goalHandle.goalId);
  expectType<boolean>(goalHandle.isActive);
  expectType<boolean>(goalHandle.isCancelRequested);
  expectType<rclnodejs.example_interfaces.action.Fibonacci_Goal>(
    goalHandle.request
  );
  expectType<string>(goalHandle.status);
  expectType<void>(goalHandle.abort());
  expectType<void>(goalHandle.canceled());
  expectType<void>(goalHandle.execute());
  expectType<void>(goalHandle.publishFeedback(new Fibonacci.Feedback()));
  expectType<void>(goalHandle.succeed());

  return new Fibonacci.Result();
}
expectType<void>(
  actionServer.configureIntrospection(
    node.getClock(),
    rclnodejs.Node.getDefaultOptions() as rclnodejs.QoS,
    rclnodejs.ServiceIntrospectionStates.CONTENTS
  )
);

// ---- ActionUuid -----
const actionUuid = new rclnodejs.ActionUuid();
expectType<rclnodejs.ActionUuid>(actionUuid);
expectType<rclnodejs.ActionUuid>(rclnodejs.ActionUuid.random());
expectType<rclnodejs.ActionUuid>(
  rclnodejs.ActionUuid.fromBytes(new Uint8Array([21, 31]))
);
expectType<string>(actionUuid.toString());
expectType<rclnodejs.unique_identifier_msgs.msg.UUID>(actionUuid.toMessage());
expectType<rclnodejs.unique_identifier_msgs.msg.UUID>(
  rclnodejs.ActionUuid.randomMessage()
);

// ---- Parameter -----
const param = rclnodejs.createMessageObject('rcl_interfaces/msg/Parameter');
expectType<rclnodejs.rcl_interfaces.msg.Parameter>(param);
param.value.integer_value = BigInt(123);
expectType<bigint>(param.value.integer_value);
param.value.byte_array_value = [1, 2, 3];
expectType<number[]>(param.value.byte_array_value);

// ---- Descriptors -----
// Note: All fields are of type string exactly equal to the type of interface.
// built-in msg
const duration = rclnodejs.createMessageObject(
  'builtin_interfaces/msg/descriptor/Duration'
);
expectType<rclnodejs.builtin_interfaces.msg.descriptor.Duration>(duration);
expectAssignable<'int32'>(duration.sec);
expectAssignable<'uint32'>(duration.nanosec);
// msg containing complex types
const poseStampedDescriptor = rclnodejs.createMessageObject(
  'geometry_msgs/msg/descriptor/PoseStamped'
);
expectType<rclnodejs.geometry_msgs.msg.descriptor.PoseStamped>(
  poseStampedDescriptor
);
expectAssignable<'std_msgs/msg/Header'>(poseStampedDescriptor.header);
expectAssignable<'geometry_msgs/msg/Pose'>(poseStampedDescriptor.pose);
// action interface
const fibonacciFeedback = rclnodejs.createMessageObject(
  'example_interfaces/action/descriptor/Fibonacci_Feedback'
);
expectType<rclnodejs.example_interfaces.action.descriptor.Fibonacci_Feedback>(
  fibonacciFeedback
);
expectAssignable<'int32[]'>(fibonacciFeedback.sequence);
// srv interface
const cancelGoalRequestDescriptor = rclnodejs.createMessageObject(
  'action_msgs/srv/descriptor/CancelGoal_Request'
);
expectType<rclnodejs.action_msgs.srv.descriptor.CancelGoal_Request>(
  cancelGoalRequestDescriptor
);
expectAssignable<'action_msgs/msg/GoalInfo'>(
  cancelGoalRequestDescriptor.goal_info
);
