# rclpy vs rclnodejs Feature Parity Status

Last updated: June 1, 2026
rclpy watermark reviewed: 8593246 (origin/rolling); rechecked June 1, 2026 after rclpy rebase. Only commit beyond prior watermark 69d5ea9 (tag 11.0.1) is 8593246, a Windows test-tolerance fix in test_async_clock.py (no user-facing API change)

Scope: tracks user-facing rclpy features reviewed from `4095493` through `69d5ea9`, plus rcl rolling APIs that both clients build on.

## Implemented (closed gaps)

| Feature                                            | rclnodejs PR/status    | Notes                                                                                                                                                                    |
| -------------------------------------------------- | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `waitForMessage()`                                 | #1444                  | One-shot subscription utility                                                                                                                                            |
| `ParameterEventHandler`                            | #1438                  | Per-parameter and event-level callbacks                                                                                                                                  |
| QoS overriding via parameters                      | #1468                  | `QoSOverridingOptions`, `QoSPolicyKind`, read-only params via `--ros-args`                                                                                               |
| Pre/post set parameter callbacks                   | #1470                  | Pipeline-based pre callbacks, post callbacks for side effects                                                                                                            |
| `MessageInfo` metadata                             | #1440                  | `rclTakeWithInfo`, `MessageInfo` class, auto-detection                                                                                                                   |
| Action server goal state guards                    | #1466                  | `publishFeedback` only in EXECUTING, `execute` guards                                                                                                                    |
| Action client feedback content filter              | #1457                  | Rolling-only native bindings, executor synchronization                                                                                                                   |
| `ParameterEventHandler` node content filter        | #1474                  | `configureNodesFilter()` applies/clears a `/parameter_events` subscription content filter for selected nodes                                                             |
| Timer `autostart` parameter                        | #1472                  | `createTimer()` accepts `autostart`; native bindings forward it and older distros emulate via cancel/reset                                                               |
| Timer `TimerInfo` in callback                      | #1472                  | Executor path injects timer metadata into JS callbacks when native support exists                                                                                        |
| `TopicEndpointInfo` type hash exposure             | Existing               | `topic_type_hash` is exposed in graph APIs and consumed by `TypeDescriptionService`                                                                                      |
| `publisher/subscription_event_type_is_supported()` | #1520 / current branch | Rolling-only (`#if ROS_VERSION >= 5000`); public JS API is `isPublisherEventTypeSupported()` / `isSubscriptionEventTypeSupported()` and guards on native-symbol presence |
| Node `enable_logger_service`                       | current branch         | `NodeOptions.enableLoggerService` starts `get_logger_levels` and `set_logger_levels` services for external logger-level control                                          |

## Skipped (not applicable)

| #   | Feature                        | Reason                                                              |
| --- | ------------------------------ | ------------------------------------------------------------------- |
| 1   | MultiThreadedExecutor          | JS is single-threaded; libuv handles concurrency                    |
| 2   | EventsExecutor                 | Experimental in rclpy                                               |
| 3b  | Callback groups                | Depends on multi-threaded executor; JS callbacks already serialized |
| 7   | Signal handler guard condition | Node.js SIGINT via libuv is already immediate                       |

## Dropped (evaluated, low value)

| #   | Feature                   | Reason                                                      |
| --- | ------------------------- | ----------------------------------------------------------- |
| 6b  | `spinUntilFutureComplete` | Syntactic sugar; `spin/await/stop` is trivial in JS (#1441) |

## Remaining gaps (low priority)

| #   | Feature                                   | Impact                                              | Effort |
| --- | ----------------------------------------- | --------------------------------------------------- | ------ |
| 12  | Waitables                                 | Low - advanced extensibility                        | High   |
| 13  | Subscription `acceptable_buffer_backends` | Low - no JS/native buffer backend interop story yet | Medium |

## Unique to rclnodejs

- ObservableSubscription (RxJS)
- Message validation (`validateMessage()`, schemas)
- Serialization modes (`'default'`, `'plain'`, `'json'`)
- Message introspection (`MessageIntrospector`, `getMessageSchema()`)
- Process launching (`ros2Run()`, `ros2Launch()`)
- JSON-safe conversion (`toJSONSafe()`, `toJSONString()`)
- ParameterWatcher (EventEmitter-based remote param watcher)

## Tracking issue

https://github.com/RobotWebTools/rclnodejs/issues/1445
