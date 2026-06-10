# rclnodejs TypeScript Topics Demo

This demo demonstrates how to use **rclnodejs** with TypeScript to create ROS2 publishers and subscribers. The demo includes a publisher that sends string messages to a topic called `ts_demo` and a subscriber that receives and displays those messages.

## Features

- 🚀 **TypeScript Support**: Fully typed ROS2 node implementation using rclnodejs TypeScript interfaces
- 📤 **Publisher**: Sends timestamped string messages at regular intervals
- 📥 **Subscriber**: Receives and displays messages with timestamps
- 🛡️ **Type Safety**: Leverages TypeScript's type system for compile-time safety
- 🎯 **Error Handling**: Comprehensive error handling and graceful shutdown
- 📊 **Message Counting**: Tracks published and received message counts
- 🎨 **Console Output**: Colorful and informative console messages

## Prerequisites

Before running this demo, ensure you have:

1. **Node.js** (>= 20.20.2)
2. **ROS 2** installed and sourced
3. **rclnodejs** built and available

### ROS 2 Setup

Make sure your ROS 2 environment is properly sourced before running the demo:

```bash
# For example, if using ROS 2 Lyrical
source /opt/ros/lyrical/setup.bash

# Or if you have a custom workspace
source /path/to/your/ros2_ws/install/setup.bash
```

### Verify rclnodejs Installation

From the root of the rclnodejs project, ensure it's built:

```bash
cd /path/to/rclnodejs
npm install
npm run build
```

## Installation

This demo depends on the **local rclnodejs** in this repository
(`"rclnodejs": "file:../../.."`) so it always builds against the in-tree
TypeScript types. Install with `--ignore-scripts` so npm links the local
package without trying to rebuild its native addon (the prebuilt binary in the
repo is reused):

```bash
cd demo/typescript/topics
npm install --ignore-scripts
```

## Usage

### Build and Run

1. **Build the TypeScript code:**

   ```bash
   npm run build
   ```

2. **Run the publisher (in one terminal):**

   ```bash
   npm run start:publisher
   ```

3. **Run the subscriber (in another terminal):**

   ```bash
   npm run start:subscriber
   ```

4. **Or run both simultaneously:**
   ```bash
   npm run start:both
   ```

### Development Mode

For development, you can run TypeScript files directly with ts-node:

1. **Run the publisher:**

   ```bash
   npm run dev:publisher
   ```

2. **Run the subscriber:**

   ```bash
   npm run dev:subscriber
   ```

3. **Type check only:**
   ```bash
   npm run check-types
   ```

## TypeScript Setup

This demo builds against the in-tree **rclnodejs** package
(`"rclnodejs": "file:../../.."`), so it uses the package's own TypeScript
declarations — there are no local stub types.

### TypeScript Configuration

The `tsconfig.json` is configured to:

- Use modern `NodeNext` module resolution, matching how Node.js resolves the
  package's `exports` map
- Target `ES2022`, consistent with the rclnodejs package itself
- Compile TypeScript to JavaScript in the `dist/` directory
- Generate source maps and declaration files
- Enable strict type checking

## Expected Output

### Publisher Output:

```
Starting TypeScript Publisher Demo...
✓ rclnodejs initialized
✓ Created node: /ts_publisher_demo
✓ Created publisher on topic: ts_demo
✓ Created timer with 1000ms interval
🚀 Publisher is running. Press Ctrl+C to stop...

📤 [A/string] Published: "Hello from TypeScript publisher! Message #1 at 2025-07-16T10:30:00.123Z"
📤 [B/class] Published: "Hello from typed publisher! Message #1"
📤 [A/string] Published: "Hello from TypeScript publisher! Message #2 at 2025-07-16T10:30:01.125Z"
📤 [B/class] Published: "Hello from typed publisher! Message #2"
...
```

### Subscriber Output:

```
Starting TypeScript Subscriber Demo...
✓ rclnodejs initialized
✓ Created node: /ts_subscriber_demo
✓ Created subscription on topic: ts_demo
👂 Subscriber is listening. Press Ctrl+C to stop...

📥 [A/string] [1] Received: "Hello from TypeScript publisher! Message #1 at 2025-07-16T10:30:00.123Z"
📥 [B/class] Received: "Hello from typed publisher! Message #1"
📥 [A/string] [2] Received: "Hello from TypeScript publisher! Message #2 at 2025-07-16T10:30:01.125Z"
📥 [B/class] Received: "Hello from typed publisher! Message #2"
...
```

## Project Structure

```
demo/typescript/topics/
├── package.json          # Project dependencies and scripts
├── tsconfig.json         # TypeScript configuration
├── README.md            # This file
├── src/                 # TypeScript source files
│   ├── publisher.ts     # Publisher implementation
│   └── subscriber.ts    # Subscriber implementation
└── dist/               # Compiled JavaScript (after build)
    ├── publisher.js
    └── subscriber.js
```

## Code Explanation

Both demos show the **two equivalent, fully type-safe ways** to identify a
message type:

- **String name** (Style A): pass the type string, e.g. `'std_msgs/msg/String'`.
- **Message class** (Style B): obtain the constructor with `rclnodejs.require(...)`
  and pass it directly. TypeScript then infers the message type from the
  constructor, so no explicit annotation is needed.

### Publisher (`src/publisher.ts`)

The publisher demonstrates:

- **Node Creation**: Creates a ROS2 node using TypeScript
- **Publisher Setup**: Creates publishers for `std_msgs/msg/String` messages
- **Timer Usage**: Uses `node.createTimer()` for periodic message publishing
- **Message Creation**: Builds messages with `createMessageObject()` (Style A)
  or by instantiating the message class (Style B)
- **Graceful Shutdown**: Handles SIGINT for clean shutdown

Key TypeScript features used:

```typescript
import * as rclnodejs from 'rclnodejs';

const node = new rclnodejs.Node('ts_publisher_demo');

// Style A — string name
const publisher = node.createPublisher('std_msgs/msg/String', TOPIC_NAME);
const message = rclnodejs.createMessageObject('std_msgs/msg/String');

// Style B — message class (type inferred from the constructor)
const StringMsg = rclnodejs.require('std_msgs/msg/String');
const typedPublisher = node.createPublisher(StringMsg, TOPIC_NAME);
const typedMessage = new StringMsg(); // `typedMessage.data` is typed
```

### Subscriber (`src/subscriber.ts`)

The subscriber demonstrates:

- **Subscription Creation**: Creates typed subscriptions for string messages
- **Callback Handling**: Processes incoming messages with proper TypeScript typing
- **Message Processing**: Displays received messages with timestamps

Key TypeScript features used:

```typescript
// Style A — string name, callback type annotated explicitly
const subscription = node.createSubscription(
  'std_msgs/msg/String',
  TOPIC_NAME,
  (message: rclnodejs.std_msgs.msg.String) => {
    console.log(`Received: "${message.data}"`);
  }
);

// Style B — message class, callback type inferred from the constructor
const StringMsg = rclnodejs.require('std_msgs/msg/String');
node.createSubscription(StringMsg, TOPIC_NAME, (message) => {
  console.log(`Received: "${message.data}"`);
});
```

## TypeScript Benefits

This demo showcases several TypeScript advantages:

1. **Type Safety**: Compile-time checking prevents common errors
2. **IntelliSense**: Better IDE support with autocompletion
3. **Interface Definitions**: Clear contract definitions for ROS2 messages
4. **Refactoring Safety**: Easier to maintain and modify code
5. **Documentation**: Types serve as inline documentation

## Troubleshooting

### Common Issues

1. **Module not found error for 'rclnodejs':**

   ```
   Cannot find module 'rclnodejs' or its corresponding type declarations
   ```

   **Solution**: Ensure you're in the correct directory and rclnodejs is properly linked:

   ```bash
   cd demo/typescript/topics
   npm install
   ```

2. **ROS 2 environment not sourced:**

   ```
   Error: Unable to find ROS 2 installation
   ```

   **Solution**: Source your ROS 2 setup file:

   ```bash
   source /opt/ros/lyrical/setup.bash
   ```

3. **Build errors:**
   ```
   TypeScript compilation errors
   ```
   **Solution**: Check TypeScript configuration and ensure all dependencies are installed:
   ```bash
   npm install
   npm run clean
   npm run build
   ```

### Debugging Tips

1. **Enable verbose logging:**

   ```bash
   export RCUTILS_LOGGING_SEVERITY=DEBUG
   ```

2. **Check ROS 2 topic list:**

   ```bash
   ros2 topic list
   ros2 topic echo /ts_demo
   ```

3. **Monitor topic info:**
   ```bash
   ros2 topic info /ts_demo
   ```

## Customization

You can easily customize this demo:

### Change Topic Name

Edit the `TOPIC_NAME` constant in both files:

```typescript
const TOPIC_NAME = 'your_custom_topic';
```

### Change Message Type

To use a different message type, update the type string or message class:

```typescript
// Style A — string name
const publisher = node.createPublisher('geometry_msgs/msg/Twist', 'cmd_vel');
const message = rclnodejs.createMessageObject('geometry_msgs/msg/Twist');

// Style B — message class (type inferred from the constructor)
const Twist = rclnodejs.require('geometry_msgs/msg/Twist');
const typedPublisher = node.createPublisher(Twist, 'cmd_vel');
const message2 = new Twist();
```

### Adjust Publishing Rate

Modify the `PUBLISH_INTERVAL` in the publisher:

```typescript
const PUBLISH_INTERVAL = 500; // Publish every 500ms
```

## Next Steps

After running this demo, you might want to explore:

1. **Different Message Types**: Try using other ROS2 message types like `geometry_msgs/msg/Twist`
2. **Services**: Implement ROS2 services with TypeScript
3. **Actions**: Create action servers and clients
4. **Parameters**: Use ROS2 parameters in your TypeScript nodes
5. **Lifecycle Nodes**: Implement managed lifecycle nodes

## Contributing

This demo is part of the rclnodejs project. To contribute:

1. Fork the rclnodejs repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This demo is licensed under the Apache License 2.0, same as the rclnodejs project.
