# rclnodejs Tutorials

This directory contains comprehensive tutorials explaining various ROS 2 features and concepts using rclnodejs. Each tutorial provides practical examples and best practices for building robust ROS 2 applications in JavaScript/Node.js.

## Getting Started

If you're new to ROS 2, start with the **Basic Concepts** tutorial to understand fundamental communication patterns, then explore other tutorials based on your specific needs.

## Available Tutorials

### 📚 Fundamentals

#### [ROS 2 Basic Concepts](ros2-basic-concepts.md)

Introduction to fundamental ROS 2 communication patterns: **Topics** (publish/subscribe) and **Services** (request/response). Essential for beginners to understand the core building blocks of ROS 2 applications.

### 🔧 Node Management

#### [Lifecycle Nodes](lifecycle-nodes.md)

Learn about managed nodes with state machines for controlled startup, shutdown, and error handling. Covers lifecycle states, transitions, and implementing robust, production-ready nodes.

#### [Parameter Service](parameter-service.md)

Dynamic configuration management using ROS 2 parameters. Covers parameter declaration, validation, runtime updates, and callbacks for building configurable applications.

### 🚀 Advanced Communication

#### [ROS 2 Actions](actionlib.md)

Implement long-running, cancellable tasks with progress feedback using the ROS 2 action system. Learn to create action servers and clients for complex robot behaviors.

#### [Service Introspection](service-introspection.md)

Debug and monitor service interactions in real-time. Learn to observe service calls, responses, and performance metrics for troubleshooting distributed systems.

#### [Content Filtering Subscription](content-filtering-subscription.md)

Reduce network traffic and improve performance by filtering messages at the DDS middleware level using SQL-like expressions. Essential for high-frequency data streams.

#### [Observable Subscriptions with RxJS](observable-subscriptions.md)

Use RxJS Observables for reactive programming with ROS 2 subscriptions. Apply operators like `throttleTime()`, `combineLatest()`, and `filter()` to process message streams declaratively.

### 🔍 Introspection & Debugging

#### [Type Description Service](type-description-service.md)

Introspect message and service types used by nodes at runtime. Learn to query type definitions, field descriptions, and dependencies for better system understanding.

## Additional Resources

- [rclnodejs API Documentation](https://robotwebtools.github.io/rclnodejs/)
- [ROS 2 Official Documentation](https://docs.ros.org/)
- [rclnodejs GitHub Repository](https://github.com/RobotWebTools/rclnodejs)
- [ROS 2 Examples](../example/)

---

_These tutorials are part of the rclnodejs project. For the latest updates and more examples, visit the [GitHub repository](https://github.com/RobotWebTools/rclnodejs)._
