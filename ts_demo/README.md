# TypeScript Demos

This directory contains TypeScript demonstrations showcasing rclnodejs functionality with full type safety and modern development practices. Each demo illustrates core ROS2 communication patterns using TypeScript interfaces and best practices.

## 📂 Available Demos

### 📨 **[topics/](./topics/)** - Publisher/Subscriber Pattern

Basic ROS2 topic communication with TypeScript type safety

- **Publisher**: Sends timestamped string messages at regular intervals
- **Subscriber**: Receives and displays messages with colorful console output
- **Features**: Message counting, error handling, graceful shutdown

### 🔧 **[services/](./services/)** - Request/Response Pattern

Synchronous service calls for immediate responses

- **Service Server**: Provides AddTwoInts service for integer addition
- **Service Client**: Makes requests with random numbers at intervals
- **Features**: Service discovery, request counting, comprehensive error handling

### ⚡ **[actions/](./actions/)** - Long-Running Task Pattern

Asynchronous actions with progress feedback and cancellation

- **Action Server**: Calculates Fibonacci sequences with progress updates
- **Action Client**: Sends goals, monitors progress, handles results
- **Features**: Goal cancellation, feedback processing, result handling

## 📊 Complexity Progression

## ️ TypeScript Features

### Type Safety

- **Full typing** for all ROS2 messages, services, and actions
- **Compile-time validation** of message structures
- **IntelliSense support** in VS Code and other TypeScript editors

### Modern Development

- **ES2020+ syntax** with async/await patterns
- **Modular architecture** with clean separation of concerns
- **Error boundaries** with comprehensive exception handling
- **Graceful shutdown** handling for SIGINT/SIGTERM

### Code Quality

- **Consistent formatting** with prettier/eslint configurations
- **Build validation** with TypeScript strict mode
- **Runtime safety** with proper error handling patterns

## 📊 Complexity Progression

| Demo         | Complexity        | Communication | Best For               |
| ------------ | ----------------- | ------------- | ---------------------- |
| **topics**   | ⭐ Basic          | Pub/Sub       | Learning fundamentals  |
| **services** | ⭐⭐ Intermediate | Req/Response  | Synchronous operations |
| **actions**  | ⭐⭐⭐ Advanced   | Goal-oriented | Long-running tasks     |

## 🔗 Related Resources

- [rclnodejs Tutorials](../tutorials/) - Comprehensive API documentation
- [Electron Demos](../electron_demo/) - GUI applications with rclnodejs
- [TypeScript Handbook](https://www.typescriptlang.org/docs/) - Language reference
- [ROS2 Documentation](https://docs.ros.org/) - Official ROS2 concepts
