# ROS 2 Services Examples

This directory contains examples demonstrating ROS 2 service-based communication using rclnodejs. These examples showcase client-server patterns for request-response communication in ROS 2.

## Overview

ROS 2 services provide a request-response communication pattern where clients send requests to services, and services process these requests and send back responses. This is different from topics which use a publish-subscribe pattern. Services are ideal for:

- Remote procedure calls
- Getting/setting configuration parameters
- Triggering actions that require confirmation
- Any communication that needs a response

## Service Examples

### AddTwoInts Service

#### Service Server (`service/service-example.js`)

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
- **Run Command**: `node example/services/service/service-example.js`

#### Service Client (`client/client-example.js`)

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
- **Run Command**: `node example/services/client/client-example.js`

#### Async Service Client (`client/async-client-example.js`)

**Purpose**: Demonstrates modern async/await patterns for service communication, solving callback hell and providing cleaner error handling.

- **Service Type**: `example_interfaces/srv/AddTwoInts`
- **Service Name**: `add_two_ints`
- **Functionality**:
  - Multiple examples showing different async patterns
  - Simple async/await calls without callbacks
  - Timeout handling with configurable timeouts
  - Request cancellation using AbortController
  - Sequential and parallel service calls
  - Comprehensive error handling
- **Features**:
  - **Modern JavaScript**: Clean async/await syntax instead of callback hell
  - **Timeout Support**: Built-in timeout with `options.timeout` (uses `AbortSignal.timeout()` internally)
  - **Cancellation**: Request cancellation using `AbortController` and `options.signal`
  - **Error Types**: Specific error types (`TimeoutError`, `AbortError`) for better error handling (async only)
  - **Backward Compatible**: Works alongside existing callback-based `sendRequest()`
  - **TypeScript Ready**: Full type safety with comprehensive TypeScript definitions
- **Run Command**: `node example/services/client/async-client-example.js`

#### Service Client Validation (`client/client-validation-example.js`)

**Purpose**: Demonstrates request validation features for service clients.

- **Service Type**: `example_interfaces/srv/AddTwoInts`
- **Service Name**: `add_two_ints`
- **Functionality**:
  - Schema introspection for service request types
  - Client-level validation with `validateRequests: true` option
  - Per-request validation override with `{ validate: true/false }`
  - Strict mode validation for detecting unknown fields
  - Async request validation with `sendRequestAsync()`
  - Error handling with `MessageValidationError`
- **Features**:
  - **Request Validation**: Catch invalid requests before sending to service
  - **Schema Introspection**: Use `getMessageSchema()` to inspect request structure
  - **Dynamic Toggle**: Enable/disable validation with `setValidation()`
  - **Detailed Errors**: Field-level validation issues with expected vs received types
  - **Strict Mode**: Detect extra fields that don't belong in the request
- **Run Command**: `node example/services/client/client-validation-example.js`
- **Note**: Standalone example - demonstrates validation errors without requiring a running service

**Key API Differences**:

```javascript
client.sendRequest(request, (response) => {
  console.log('Response:', response.sum);
});

try {
  const response = await client.sendRequestAsync(request);

  const response = await client.sendRequestAsync(request, { timeout: 5000 });

  const controller = new AbortController();
  const response = await client.sendRequestAsync(request, {
    signal: controller.signal,
  });

  const controller = new AbortController();
  const response = await client.sendRequestAsync(request, {
    timeout: 5000,
    signal: controller.signal,
  });

  console.log('Response:', response.sum);
} catch (error) {
  if (error.name === 'TimeoutError') {
    console.log('Request timed out');
  } else if (error.name === 'AbortError') {
    console.log('Request was cancelled');
  } else {
    console.error('Service error:', error.message);
  }
}
```

### GetMap Service

#### Service Server (`service/getmap-service-example.js`)

**Purpose**: Demonstrates creating a service server that provides occupancy grid map data.

- **Service Type**: `nav_msgs/srv/GetMap`
- **Service Name**: `get_map`
- **Functionality**:
  - Provides a sample 10x10 occupancy grid map
  - Returns map metadata including resolution, origin, and frame_id
  - Includes realistic map data with free space and obstacles
  - Updates timestamp on each request
- **Features**:
  - Complex message types (OccupancyGrid with nested structures)
  - Realistic navigation map data for robotics applications
  - Service introspection support (ROS 2 Iron+)
  - Detailed logging of map properties
- **Run Command**: `node example/services/service/getmap-service-example.js`

#### Service Client (`client/getmap-client-example.js`)

**Purpose**: Demonstrates creating a service client that requests map data from the GetMap service.

- **Service Type**: `nav_msgs/srv/GetMap`
- **Service Name**: `get_map`
- **Functionality**:
  - Sends empty request (GetMap requests have no parameters)
  - Receives and analyzes occupancy grid map data
  - Displays comprehensive map information and statistics
  - Shows ASCII visualization for small maps
- **Features**:
  - Handling complex ROS 2 message types (OccupancyGrid)
  - Map data analysis (cell distribution, metadata extraction)
  - Educational output showing map properties
  - Visual representation of map data
- **Run Command**: `node example/services/client/getmap-client-example.js`

## How to Run the Examples

### Running the AddTwoInts Service Example

1. **Prerequisites**: Ensure ROS 2 is installed and sourced

2. **Start the Service Server**: In one terminal, run:

   ```bash
   cd /path/to/rclnodejs
   node example/services/service/service-example.js
   ```

   You should see:

   ```
   Introspection configured
   ```

3. **Start the Client**: In another terminal, run:

   ```bash
   cd /path/to/rclnodejs
   node example/services/client/client-example.js
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

### Running the Async AddTwoInts Client Example

1. **Prerequisites**: Ensure ROS 2 is installed and sourced

2. **Start the Service Server**: Use the same service server as above:

   ```bash
   cd /path/to/rclnodejs
   node example/services/service/service-example.js
   ```

3. **Start the Async Client**: In another terminal, run:

   ```bash
   cd /path/to/rclnodejs
   node example/services/client/async-client-example.js
   ```

4. **Expected Output**:

   **Service Server Terminal**: (Same as regular client)

   ```
   Incoming request: object { a: 42n, b: 37n }
   Sending response: object { sum: 79n }
   --
   ```

   **Async Client Terminal**:

   ```
   Sending: object { a: 42n, b: 37n }
   Result: object { sum: 79n }
   ```

### Running the GetMap Service Example

1. **Prerequisites**: Ensure ROS 2 is installed and sourced

2. **Start the GetMap Service Server**: In one terminal, run:

   ```bash
   cd /path/to/rclnodejs
   node example/services/service/getmap-service-example.js
   ```

   You should see:

   ```
   Introspection configured
   GetMap service is ready. Waiting for requests...
   Service provides a 10x10 occupancy grid map
   ```

3. **Start the GetMap Client**: In another terminal, run:

   ```bash
   cd /path/to/rclnodejs
   node example/services/client/getmap-client-example.js
   ```

4. **Expected Output**:

   **Service Server Terminal**:

   ```
   Incoming GetMap request: object {}
   Sending map with dimensions: 10 x 10
   Map resolution: 0.05 meters/pixel
   Number of occupied cells: 3
   Sending response: object Map data size: 100
   --
   ```

   **Client Terminal**:

   ```
   GetMap service found. Requesting map...
   Sending: object {}
   Response received: object
   Map metadata:
     Frame ID: map
     Timestamp: 1756479378.889000000
     Resolution: 0.05000000074505806 meters/pixel
     Dimensions: 10x10
     Origin position: x: -2.5 y: -2.5 z: 0
     Map data size: 100 cells
     Cell distribution:
       Free cells (0): 97
       Occupied cells (100): 3
       Unknown cells (-1): 0

   ASCII Map Representation:
     . = free space, # = occupied, ? = unknown
     ..........
     ..........
     ..#.......
     ...#......
     ....#.....
     ..........
     ..........
     ..........
     ..........
     ..........
   ```

### Quick Test Script

For convenience, you can test the GetMap examples using the provided test script from the project root:

```bash
cd /path/to/rclnodejs
./test_getmap_examples.sh
```

This script automatically starts the service, tests the client, and cleans up.

### Single Run vs Continuous Operation

- **Client**: Runs once, sends a request, receives response, then shuts down
- **Service**: Runs continuously, waiting for requests until manually terminated (Ctrl+C)

## Key Concepts Demonstrated

### Service Communication Pattern

- **Request-Response**: Synchronous communication where clients wait for responses
- **Service Discovery**: Clients check if services are available before sending requests
- **Error Handling**: Proper handling of service unavailability
- **Empty Requests**: Services like GetMap that don't require input parameters
- **Complex Responses**: Handling nested message structures like OccupancyGrid

### ROS 2 Service Features

- **BigInt Support**: Using JavaScript BigInt for ROS 2 integer types
- **Service Introspection**: Monitoring service calls and events (ROS 2 Iron+)
- **Quality of Service**: Configurable QoS profiles for reliable communication
- **Complex Message Types**: Working with navigation messages (nav_msgs)
- **Nested Structures**: Handling messages with headers, metadata, and arrays

### Programming Patterns

- **Modern Async/Await**: Clean Promise-based service calls with `sendRequestAsync()`
- **Traditional Callbacks**: Response processing using callback functions with `sendRequest()`
- **Error Handling**: Proper error handling with try/catch blocks and specific error types (async only)
- **Timeout Management**: Built-in timeout support to prevent hanging requests (async only)
- **Request Cancellation**: AbortController support for user-cancellable operations (async only)
- **Request Validation**: Pre-send validation with `validateRequests` option and `MessageValidationError`
- **Schema Introspection**: Programmatic access to service request/response schemas
- **Resource Management**: Proper node shutdown and cleanup
- **Data Analysis**: Processing and interpreting received data
- **Visualization**: Converting data to human-readable formats

### Robotics Applications

- **Navigation**: Map data retrieval for path planning and localization
- **Occupancy Grids**: Understanding spatial data representation
- **Coordinate Systems**: Working with map origins, resolutions, and frames
- **Sensor Data**: Processing grid-based sensor information

## Service Introspection (ROS 2 Iron+)

All examples include service introspection capabilities for ROS 2 distributions newer than Humble:

### AddTwoInts Service

- **Service Server**: Configured with `CONTENTS` introspection to log full request/response data
- **Client**: Configured with `METADATA` introspection to log service call metadata

To monitor AddTwoInts service events:

```bash
ros2 topic echo "/add_two_ints/_service_event"
```

### GetMap Service

- **Service Server**: Configured with `CONTENTS` introspection to log full request/response data
- **Client**: Configured with `METADATA` introspection to log service call metadata

To monitor GetMap service events:

```bash
ros2 topic echo "/get_map/_service_event"
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

**JavaScript Implementation Details**:

- **BigInt Usage**: ROS 2 `int64` maps to JavaScript `BigInt` type
- **Response Template**: Use `response.template` to get the proper response structure
- **Response Sending**: Call `response.send(result)` to send the response back

### GetMap Service Definition

```
# Request
# Empty - no parameters needed
---
# Response
nav_msgs/OccupancyGrid map
```

**OccupancyGrid Structure**:

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
MapMetaData info
  builtin_interfaces/Time map_load_time
  float32 resolution
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
int8[] data
```

**JavaScript Implementation Details**:

- **Empty Requests**: GetMap requests are empty objects `{}`
- **Complex Responses**: OccupancyGrid contains nested headers, metadata, and data arrays
- **Map Data**: Array of int8 values (0=free, 100=occupied, -1=unknown)
- **Coordinate Systems**: Origin defines the real-world position of the map's (0,0) cell

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
   - For async clients, use timeout options: `client.sendRequestAsync(request, { timeout: 5000 })`

4. **Async/Await Issues** (applies only to `sendRequestAsync()`):
   - **Unhandled Promise Rejections**: Always use try/catch blocks around `sendRequestAsync()`
   - **Timeout Errors**: Handle `TimeoutError` specifically for timeout scenarios (async only)
   - **Cancelled Requests**: Handle `AbortError` when using AbortController cancellation (async only)
   - **Mixed Patterns**: You can use both `sendRequest()` and `sendRequestAsync()` in the same code

### Debugging Tips

- Use `ros2 service list` to see available services
- Use `ros2 service type <service_name>` to check service types
- Use `ros2 service call <service_name> <service_type> <request>` to test services from command line

## Notes

- Both examples use the standard rclnodejs initialization pattern
- The service server runs continuously until manually terminated
- The traditional client performs a single request-response cycle then exits
- The async client demonstrates multiple patterns and then exits
- **New async/await support**: Use `sendRequestAsync()` for modern Promise-based patterns
- **Full backward compatibility**: Existing `sendRequest()` callback-based code continues to work unchanged
- **TypeScript support**: Full type safety available for async methods
- Service introspection is only available in ROS 2 Iron and later distributions
- BigInt is required for integer message fields to maintain precision
