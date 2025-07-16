# rclnodejs TypeScript Services Demo

This demo demonstrates how to use **rclnodejs** with TypeScript to create ROS2 service servers and clients. The demo includes a service server that provides an `AddTwoInts` service and a client that calls the service with random numbers.

## Features

- 🚀 **TypeScript Support**: Fully typed ROS2 service implementation using rclnodejs TypeScript interfaces
- 🔢 **Service Server**: Provides an AddTwoInts service that adds two integers
- 📞 **Service Client**: Calls the AddTwoInts service with random numbers at regular intervals
- 🛡️ **Type Safety**: Leverages TypeScript's type system for compile-time safety
- 🎯 **Error Handling**: Comprehensive error handling and graceful shutdown
- 📊 **Request Counting**: Tracks service requests and responses
- 🎨 **Console Output**: Colorful and informative console messages
- ⏱️ **Service Discovery**: Client waits for service availability before making requests

## Prerequisites

Before running this demo, ensure you have:

1. **Node.js** (>= 16.13.0)
2. **ROS 2** installed and sourced
3. **rclnodejs** built and available

### ROS 2 Setup

Make sure your ROS 2 environment is properly sourced before running the demo:

```bash
# For example, if using ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

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

Navigate to this demo directory and install dependencies:

```bash
cd ts_demo/services
npm install
```

## Usage

### Build and Run

1. **Build the TypeScript code:**

   ```bash
   npm run build
   ```

2. **Run the service server (in one terminal):**

   ```bash
   npm run start:server
   ```

3. **Run the service client (in another terminal):**

   ```bash
   npm run start:client
   ```

4. **Or run both simultaneously:**
   ```bash
   npm run start:both
   ```

### Development Mode

For development, you can run TypeScript files directly with ts-node:

1. **Run the service server:**

   ```bash
   npm run dev:server
   ```

2. **Run the service client:**

   ```bash
   npm run dev:client
   ```

3. **Type check only:**
   ```bash
   npm run check-types
   ```

## How It Works

### Service Server (`server.ts`)

The service server:

- Creates a ROS2 node named `ts_server_demo`
- Provides an `AddTwoInts` service at `/add_two_ints`
- Receives requests with two integers (`a` and `b`)
- Calculates the sum and returns it in the response
- Logs each request and response with timestamps

### Service Client (`client.ts`)

The service client:

- Creates a ROS2 node named `ts_client_demo`
- Waits for the `AddTwoInts` service to become available
- Sends requests every 3 seconds with random integers
- Displays the results and verifies the calculations
- Handles service availability and error cases

### Example Output

**Service Server:**

```
Starting TypeScript Service Server Demo...
✓ rclnodejs initialized
✓ Created node: /mock_namespace/ts_server_demo
✓ Created service server: add_two_ints
🚀 Service server is running. Press Ctrl+C to stop...

🔢 [1] Received request: a=42, b=17
📤 [1] Sending response: sum=59
    Calculation: 42 + 17 = 59
    Timestamp: 2025-07-16T02:30:15.123Z
```

**Service Client:**

```
Starting TypeScript Service Client Demo...
✓ rclnodejs initialized
✓ Created node: /mock_namespace/ts_client_demo
✓ Created service client: add_two_ints
⏳ Waiting for service to be available...
✓ Service is available
🚀 Starting to send requests. Press Ctrl+C to stop...

📞 [1] Sending request: a=42, b=17
    Timestamp: 2025-07-16T02:30:15.120Z
📨 [1] Received response: sum=59
    Verification: 42 + 17 = 59 ✓
    Response time: 2025-07-16T02:30:15.125Z
```

## TypeScript Configuration

This demo includes:

- **Local Type Definitions**: Custom TypeScript definitions for rclnodejs in `types/rclnodejs.d.ts`
- **Service Types**: Type definitions for `example_interfaces/srv/AddTwoInts`
- **Strict Type Checking**: Full TypeScript strict mode enabled
- **Build Pipeline**: Automated compilation with shebang fixing for executable JavaScript

## Service Interface

The `AddTwoInts` service interface:

```typescript
export interface AddTwoInts {
  Request: {
    a: number; // First integer
    b: number; // Second integer
  };
  Response: {
    sum: number; // Sum of a and b
  };
}
```

## Customization

You can easily customize this demo:

### Change Service Name

Edit the `SERVICE_NAME` constant in both files:

```typescript
const SERVICE_NAME = 'your_custom_service';
```

### Change Request Interval

Modify the `REQUEST_INTERVAL` in the client:

```typescript
const REQUEST_INTERVAL = 1000; // Send requests every 1 second
```

### Use Different Service Types

To use a different service type, update the type string and interface:

```typescript
// For example, using a custom service
const service = node.createService(
  'your_package/srv/YourService',
  'service_name',
  callback
);
```

### Add Custom Logic

Modify the service callback to implement your own business logic:

```typescript
(request, response) => {
  // Your custom service logic here
  response.result = processRequest(request);
};
```

## Troubleshooting

### Common Issues

1. **Service not available:**

   ```
   Service not available after 5 seconds
   ```

   **Solution**: Make sure the service server is running before starting the client.

2. **Module not found error:**

   ```
   Cannot find module 'rclnodejs'
   ```

   **Solution**: Ensure rclnodejs is properly linked and you're in the correct directory.

3. **TypeScript compilation errors:**
   ```
   Type errors in service definitions
   ```
   **Solution**: Check the type definitions in `types/rclnodejs.d.ts` match your usage.

### Debugging Tips

1. **Check service list:**

   ```bash
   ros2 service list
   ros2 service type /add_two_ints
   ```

2. **Call service manually:**

   ```bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
   ```

3. **Monitor service info:**
   ```bash
   ros2 service info /add_two_ints
   ```

## Next Steps

After running this demo, you might want to explore:

1. **Custom Service Types**: Create your own service definitions
2. **Asynchronous Services**: Implement long-running service operations
3. **Service Parameters**: Use ROS2 parameters in your services
4. **Service Discovery**: Implement dynamic service discovery
5. **Lifecycle Services**: Create managed lifecycle service nodes

## License

This demo is licensed under the Apache License 2.0, same as the rclnodejs project.
