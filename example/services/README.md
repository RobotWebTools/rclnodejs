# ROS 2 Services Examples

This directory contains examples demonstrating ROS 2 service-based communication using rclnodejs. These examples showcase client-server patterns for request-response communication in ROS 2.

## Overview

ROS 2 services provide a request-response communication pattern where clients send requests to services, and services process these requests and send back responses. This is different from topics which use a publish-subscribe pattern. Services are ideal for:

- Remote procedure calls
- Getting/setting configuration parameters
- Triggering actions that require confirmation
- Any communication that needs a response

## Service Examples

### Service Server (`service/service-example.js`)

**Purpose**: Demonstrates creating a service server that adds two integers.

- **Service Type**: `example_interfaces/srv/AddTwoInts`
- **Service Name**: `add_two_ints`
- **Functionality**:
  - Receives requests with two BigInt numbers (`a` and `b`)
  - Computes the sum (`a + b`)
  - Sends back the result in the response
  - Logs incoming requests and outgoing responses
- **Features**:
  - Service introspection (ROS 2 Iron+) for monitoring service calls
  - Proper response handling using `response.template` and `response.send()`
- **Run Command**: `node service/service-example.js`

### Service Client (`client/client-example.js`)

**Purpose**: Demonstrates creating a service client that sends requests to the AddTwoInts service.

- **Service Type**: `example_interfaces/srv/AddTwoInts`
- **Service Name**: `add_two_ints`
- **Functionality**:
  - Generates two random BigInt numbers (0-99)
  - Sends a request to the service with these numbers
  - Waits for and logs the response
  - Automatically shuts down after receiving the response
- **Features**:
  - Service availability checking with `waitForService()`
  - Service introspection configuration (ROS 2 Iron+)
  - Asynchronous request handling with callbacks
- **Run Command**: `node client/client-example.js`

## How to Run the Examples

### Running the Complete Service Example

1. **Prerequisites**: Ensure ROS 2 is installed and sourced

2. **Start the Service Server**: In one terminal, run:

   ```bash
   cd example/services
   node service/service-example.js
   ```

   You should see:

   ```
   Introspection configured
   ```

3. **Start the Client**: In another terminal, run:

   ```bash
   cd example/services
   node client/client-example.js
   ```

4. **Expected Output**:

   **Service Server Terminal**:

   ```
   Incoming request: object { a: 42n, b: 37n }
   Sending response: object { sum: 79n }
   --
   ```

   **Client Terminal**:

   ```
   Sending: object { a: 42n, b: 37n }
   Result: object { sum: 79n }
   ```

### Single Run vs Continuous Operation

- **Client**: Runs once, sends a request, receives response, then shuts down
- **Service**: Runs continuously, waiting for requests until manually terminated (Ctrl+C)

## Key Concepts Demonstrated

### Service Communication Pattern

- **Request-Response**: Synchronous communication where clients wait for responses
- **Service Discovery**: Clients check if services are available before sending requests
- **Error Handling**: Proper handling of service unavailability

### ROS 2 Service Features

- **BigInt Support**: Using JavaScript BigInt for ROS 2 integer types
- **Service Introspection**: Monitoring service calls and events (ROS 2 Iron+)
- **Quality of Service**: Configurable QoS profiles for reliable communication

### Programming Patterns

- **Async/Await**: Modern JavaScript patterns for asynchronous operations
- **Callback Handling**: Response processing using callback functions
- **Resource Management**: Proper node shutdown and cleanup

## Service Introspection (ROS 2 Iron+)

Both examples include service introspection capabilities for ROS 2 distributions newer than Humble:

- **Service Server**: Configured with `CONTENTS` introspection to log full request/response data
- **Client**: Configured with `METADATA` introspection to log service call metadata

To monitor service events, use:

```bash
ros2 topic echo "/add_two_ints/_service_event"
```

## Message Types and Data Handling

### AddTwoInts Service Definition

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### JavaScript Implementation Details

- **BigInt Usage**: ROS 2 `int64` maps to JavaScript `BigInt` type
- **Response Template**: Use `response.template` to get the proper response structure
- **Response Sending**: Call `response.send(result)` to send the response back

## Troubleshooting

### Common Issues

1. **Service Not Available**:

   - Ensure the service server is running before starting the client
   - Check that both use the same service name (`add_two_ints`)

2. **Type Errors**:

   - Ensure you're using `BigInt()` for integer values, not regular numbers
   - Use `response.template` to get the correct response structure

3. **Client Hangs**:
   - The client waits for service availability with a 1-second timeout
   - If the service isn't available, the client will log an error and shut down

### Debugging Tips

- Use `ros2 service list` to see available services
- Use `ros2 service type <service_name>` to check service types
- Use `ros2 service call <service_name> <service_type> <request>` to test services from command line

## Notes

- Both examples use the standard rclnodejs initialization pattern
- The service server runs continuously until manually terminated
- The client performs a single request-response cycle then exits
- Service introspection is only available in ROS 2 Iron and later distributions
- BigInt is required for integer message fields to maintain precision
