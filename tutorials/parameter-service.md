# ROS 2 Parameter Service Tutorial

This tutorial explains how to use ROS 2 parameter services in rclnodejs for dynamic configuration management, parameter validation, and runtime parameter updates.

## Table of Contents

- [What are ROS 2 Parameters?](#what-are-ros-2-parameters)
- [Parameter Types and Components](#parameter-types-and-components)
- [Basic Parameter Operations](#basic-parameter-operations)
- [Parameter Descriptors](#parameter-descriptors)
- [Parameter Service Implementation](#parameter-service-implementation)
- [Parameter Callbacks and Validation](#parameter-callbacks-and-validation)
- [Parameter Overrides](#parameter-overrides)
- [Advanced Features](#advanced-features)
- [Best Practices](#best-practices)
- [Running the Examples](#running-the-examples)

## What are ROS 2 Parameters?

**ROS 2 Parameters** are a powerful mechanism for **dynamic node configuration** that allows you to:

- 🔧 **Configure node behavior** at runtime without code changes
- 📊 **Store typed values** (strings, integers, doubles, booleans, arrays)
- 🔒 **Validate parameters** with descriptors and callbacks
- 🌐 **Share configuration** across nodes and launch files
- 📝 **Override values** via command line or launch parameters
- 🏷️ **Address hierarchically** using node name, node namespace, parameter name, and parameter namespace

Parameters provide a standardized way to manage configuration that's discoverable, type-safe, and remotely accessible through ROS 2 services. Each parameter consists of a **key** (string), **value** (typed data), and **descriptor** (metadata including type information, ranges, and constraints).

### Parameter Lifetime and Addressing

Parameters in ROS 2 are:

- **Tied to node lifetime** - Parameters exist as long as the node exists
- **Addressed hierarchically** - Using node name, node namespace, parameter name, and optional parameter namespace
- **Declared by default** - Nodes must declare parameters they will accept (for type safety)

## Parameter Types and Components

### Parameter Types

rclnodejs supports all ROS 2 parameter types:

```javascript
const rclnodejs = require('rclnodejs');

// Parameter type enumeration
const ParameterType = rclnodejs.ParameterType;

// Available types:
// PARAMETER_BOOL - boolean values
// PARAMETER_INTEGER - 64-bit integers (BigInt)
// PARAMETER_DOUBLE - double precision floating point
// PARAMETER_STRING - string values
// PARAMETER_BYTE_ARRAY - array of bytes
// PARAMETER_BOOL_ARRAY - array of booleans
// PARAMETER_INTEGER_ARRAY - array of integers
// PARAMETER_DOUBLE_ARRAY - array of doubles
// PARAMETER_STRING_ARRAY - array of strings
```

### Core Classes

```javascript
const Parameter = rclnodejs.Parameter;
const ParameterDescriptor = rclnodejs.ParameterDescriptor;
const FloatingPointRange = rclnodejs.FloatingPointRange;
const IntegerRange = rclnodejs.IntegerRange;
```

## Basic Parameter Operations

### Parameter Declaration Requirements

**Important**: By default, ROS 2 nodes must declare all parameters they will accept during their lifetime. This design ensures:

- **Type safety** - Parameter types are well-defined at startup
- **Name validation** - Parameter names are known and validated
- **Reduced misconfiguration** - Prevents runtime errors from typos or wrong types
- **Better introspection** - Tools can discover available parameters

```javascript
async function demonstrateDeclarationRequirement() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('my_node');

  // ❌ This will fail - parameter not declared
  const ParameterType = rclnodejs.ParameterType;
  const Parameter = rclnodejs.Parameter;

  const result1 = node.setParameter(
    new Parameter('undeclared_param', ParameterType.PARAMETER_STRING, 'value')
  );
  console.log(`Undeclared parameter result: ${result1.successful}`); // false

  // ✅ This works - parameter properly declared first
  node.declareParameter(
    new Parameter('my_param', ParameterType.PARAMETER_STRING, 'default')
  );
  const result2 = node.setParameter(
    new Parameter('my_param', ParameterType.PARAMETER_STRING, 'new_value')
  );
  console.log(`Declared parameter result: ${result2.successful}`); // true

  rclnodejs.shutdown();
}
```

### Parameter Declaration

Parameters must be declared before use:

```javascript
const rclnodejs = require('rclnodejs');
const ParameterType = rclnodejs.ParameterType;
const Parameter = rclnodejs.Parameter;
const ParameterDescriptor = rclnodejs.ParameterDescriptor;

async function basicParameterExample() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('parameter_demo_node');

  // Create a string parameter
  const stringParam = new Parameter(
    'robot_name',
    ParameterType.PARAMETER_STRING,
    'my_robot'
  );

  // Create a parameter descriptor
  const stringDescriptor = new ParameterDescriptor(
    'robot_name',
    ParameterType.PARAMETER_STRING,
    'Name of the robot'
  );

  // Declare the parameter
  node.declareParameter(stringParam, stringDescriptor);
  console.log(`Declared parameter: ${stringParam.name}`);

  // Check if parameter exists
  if (node.hasParameter('robot_name')) {
    const param = node.getParameter('robot_name');
    console.log(`Parameter value: ${param.value}`);
  }

  rclnodejs.shutdown();
}

basicParameterExample();
```

### Multiple Parameter Types

```javascript
class ParameterDemoNode {
  constructor() {
    this.node = rclnodejs.createNode('multi_param_node');
    this.declareAllParameters();
  }

  declareAllParameters() {
    // Boolean parameter
    const enableLogging = new Parameter(
      'enable_logging',
      ParameterType.PARAMETER_BOOL,
      true
    );

    // Integer parameter (note: BigInt for integers)
    const maxConnections = new Parameter(
      'max_connections',
      ParameterType.PARAMETER_INTEGER,
      10n
    );

    // Double parameter
    const updateRate = new Parameter(
      'update_rate',
      ParameterType.PARAMETER_DOUBLE,
      50.0
    );

    // String array parameter
    const topics = new Parameter(
      'subscribed_topics',
      ParameterType.PARAMETER_STRING_ARRAY,
      ['/sensor/data', '/robot/status']
    );

    // Declare all parameters
    const parameters = [enableLogging, maxConnections, updateRate, topics];
    const descriptors = [
      new ParameterDescriptor('enable_logging', ParameterType.PARAMETER_BOOL),
      new ParameterDescriptor(
        'max_connections',
        ParameterType.PARAMETER_INTEGER
      ),
      new ParameterDescriptor('update_rate', ParameterType.PARAMETER_DOUBLE),
      new ParameterDescriptor(
        'subscribed_topics',
        ParameterType.PARAMETER_STRING_ARRAY
      ),
    ];

    this.node.declareParameters(parameters, descriptors);
    console.log('All parameters declared successfully');
  }

  getParameterValues() {
    // Get individual parameter
    const enableLogging = this.node.getParameter('enable_logging');
    console.log(`Logging enabled: ${enableLogging.value}`);

    // Get multiple parameters at once
    const paramNames = ['max_connections', 'update_rate'];
    const params = this.node.getParameters(paramNames);
    params.forEach((param) => {
      console.log(`${param.name}: ${param.value}`);
    });

    // Get all parameter names
    const allNames = this.node.getParameterNames();
    console.log(`Declared parameters: ${allNames.join(', ')}`);
  }
}
```

## Parameter Descriptors

### Basic Descriptors

```javascript
// Simple descriptor
const basicDescriptor = new ParameterDescriptor(
  'my_param',
  ParameterType.PARAMETER_STRING,
  'A simple string parameter'
);

// Read-only parameter
const readOnlyDescriptor = new ParameterDescriptor(
  'version',
  ParameterType.PARAMETER_STRING,
  'Software version',
  true // read-only
);
```

### Descriptors with Ranges

```javascript
// Integer parameter with range validation
const IntegerRange = rclnodejs.IntegerRange;
const integerDescriptor = new ParameterDescriptor(
  'port_number',
  ParameterType.PARAMETER_INTEGER,
  'Network port number'
);
integerDescriptor.range = new IntegerRange(1024n, 65535n, 1n);

// Double parameter with range validation
const FloatingPointRange = rclnodejs.FloatingPointRange;
const doubleDescriptor = new ParameterDescriptor(
  'frequency',
  ParameterType.PARAMETER_DOUBLE,
  'Update frequency in Hz'
);
doubleDescriptor.range = new FloatingPointRange(0.1, 100.0, 0.1);

// Declare parameters with validation
node.declareParameter(
  new Parameter('port_number', ParameterType.PARAMETER_INTEGER, 8080n),
  integerDescriptor
);

node.declareParameter(
  new Parameter('frequency', ParameterType.PARAMETER_DOUBLE, 10.0),
  doubleDescriptor
);
```

## Parameter Service Implementation

### Starting Parameter Services

The parameter service provides standardized ROS 2 interfaces for parameter operations:

```javascript
class ParameterServiceNode {
  constructor() {
    // Parameter services are enabled by default
    // You can explicitly control them via NodeOptions
    const NodeOptions = rclnodejs.NodeOptions;
    const options = new NodeOptions();
    options.startParameterServices = true; // default is true

    this.node = rclnodejs.createNode('param_service_node', options);

    // Declare some parameters
    this.declareParameters();

    console.log('Parameter services are running');
  }

  declareParameters() {
    const parameters = [
      new Parameter('device_name', ParameterType.PARAMETER_STRING, 'sensor_1'),
      new Parameter('sample_rate', ParameterType.PARAMETER_DOUBLE, 100.0),
      new Parameter('enable_debug', ParameterType.PARAMETER_BOOL, false),
    ];

    const descriptors = parameters.map(
      (param) =>
        new ParameterDescriptor(
          param.name,
          param.type,
          `Parameter: ${param.name}`
        )
    );

    this.node.declareParameters(parameters, descriptors);
  }

  async run() {
    console.log('Node running with parameter services...');
    rclnodejs.spin(this.node);
  }
}

// Usage
async function main() {
  await rclnodejs.init();
  const paramNode = new ParameterServiceNode();
  await paramNode.run();
}

main().catch(console.error);
```

### Service Interfaces

The parameter service automatically creates these ROS 2 service endpoints for external parameter access:

- **`/node_name/list_parameters`** - `rcl_interfaces/srv/ListParameters`

  - Lists available parameters with optional prefix filtering
  - Returns parameter names and prefixes based on depth settings

- **`/node_name/get_parameters`** - `rcl_interfaces/srv/GetParameters`

  - Gets parameter values for specified parameter names
  - Returns current values for existing parameters

- **`/node_name/get_parameter_types`** - `rcl_interfaces/srv/GetParameterTypes`

  - Gets parameter types for specified parameter names
  - Returns type information for introspection

- **`/node_name/describe_parameters`** - `rcl_interfaces/srv/DescribeParameters`

  - Gets parameter descriptors for specified parameter names
  - Returns metadata including descriptions, constraints, and ranges

- **`/node_name/set_parameters`** - `rcl_interfaces/srv/SetParameters`

  - Sets multiple parameters individually
  - Returns results for each parameter (some may succeed, others fail)

- **`/node_name/set_parameters_atomically`** - `rcl_interfaces/srv/SetParametersAtomically`
  - Sets multiple parameters as a single atomic operation
  - Returns single result (all succeed or all fail)

These services enable external tools like `ros2 param` to interact with node parameters.

## Parameter Callbacks and Validation

### Parameter Change Monitoring

rclnodejs implements the **"set parameter" callback** from the ROS 2 parameter callback system. This callback:

- **Validates parameter changes** before they are applied
- **Can reject changes** by returning `{successful: false, reason: "error message"}`
- **Receives immutable parameter objects** to inspect proposed changes
- **Should have no side effects** - changes may still be rejected by other callbacks
- **Executes before parameters are actually updated**

**Note**: The full ROS 2 specification defines three callback types:

- Pre-set parameter callbacks (not available in rclnodejs)
- **Set parameter callbacks** (✅ available in rclnodejs via `addOnSetParametersCallback`)
- Post-set parameter callbacks (not available in rclnodejs)

Monitor and validate parameter changes:

```javascript
class ValidatedParameterNode {
  constructor() {
    this.node = rclnodejs.createNode('validated_param_node');
    this.setupParameters();
    this.addParameterCallbacks();
  }

  setupParameters() {
    // Declare parameters with validation
    const params = [
      new Parameter('max_speed', ParameterType.PARAMETER_DOUBLE, 2.0),
      new Parameter('device_id', ParameterType.PARAMETER_STRING, 'default'),
      new Parameter('safety_enabled', ParameterType.PARAMETER_BOOL, true),
    ];

    const descriptors = [
      new ParameterDescriptor(
        'max_speed',
        ParameterType.PARAMETER_DOUBLE,
        'Maximum speed in m/s'
      ),
      new ParameterDescriptor(
        'device_id',
        ParameterType.PARAMETER_STRING,
        'Device identifier'
      ),
      new ParameterDescriptor(
        'safety_enabled',
        ParameterType.PARAMETER_BOOL,
        'Enable safety systems'
      ),
    ];

    // Add range validation for speed
    const FloatingPointRange = rclnodejs.FloatingPointRange;
    descriptors[0].range = new FloatingPointRange(0.0, 5.0, 0.1);

    this.node.declareParameters(params, descriptors);
  }

  addParameterCallbacks() {
    // Add parameter validation callback
    this.node.addOnSetParametersCallback((parameters) => {
      console.log(`Validating ${parameters.length} parameter(s)...`);

      for (const param of parameters) {
        // Custom validation logic
        if (param.name === 'device_id' && param.value === 'invalid') {
          console.log(`Rejecting invalid device_id: ${param.value}`);
          return {
            successful: false,
            reason: 'Invalid device identifier',
          };
        }

        if (param.name === 'max_speed' && param.value < 0) {
          console.log(`Rejecting negative speed: ${param.value}`);
          return {
            successful: false,
            reason: 'Speed cannot be negative',
          };
        }

        console.log(`Accepting parameter ${param.name} = ${param.value}`);
      }

      return { successful: true, reason: '' };
    });
  }

  updateParameters() {
    // Valid parameter update
    const newSpeed = new Parameter(
      'max_speed',
      ParameterType.PARAMETER_DOUBLE,
      1.5
    );
    const result = this.node.setParameter(newSpeed);

    if (result.successful) {
      console.log('Speed parameter updated successfully');
    } else {
      console.log(`Failed to update speed: ${result.reason}`);
    }

    // Invalid parameter update (will be rejected)
    const invalidDevice = new Parameter(
      'device_id',
      ParameterType.PARAMETER_STRING,
      'invalid'
    );
    const invalidResult = this.node.setParameter(invalidDevice);

    if (!invalidResult.successful) {
      console.log(`Parameter rejected: ${invalidResult.reason}`);
    }
  }
}
```

### Atomic Parameter Updates

```javascript
class AtomicParameterNode {
  constructor() {
    this.node = rclnodejs.createNode('atomic_param_node');
    this.setupParameters();
  }

  setupParameters() {
    const params = [
      new Parameter('width', ParameterType.PARAMETER_INTEGER, 100n),
      new Parameter('height', ParameterType.PARAMETER_INTEGER, 100n),
      new Parameter('aspect_ratio', ParameterType.PARAMETER_DOUBLE, 1.0),
    ];

    this.node.declareParameters(params);

    // Add validation to maintain aspect ratio
    this.node.addOnSetParametersCallback((parameters) => {
      const paramMap = new Map();
      parameters.forEach((p) => paramMap.set(p.name, p.value));

      // If width or height changes, validate aspect ratio
      if (paramMap.has('width') || paramMap.has('height')) {
        const width =
          paramMap.get('width') || this.node.getParameter('width').value;
        const height =
          paramMap.get('height') || this.node.getParameter('height').value;
        const expectedRatio = Number(width) / Number(height);

        if (paramMap.has('aspect_ratio')) {
          const setRatio = paramMap.get('aspect_ratio');
          if (Math.abs(setRatio - expectedRatio) > 0.01) {
            return {
              successful: false,
              reason: 'Aspect ratio must match width/height ratio',
            };
          }
        }
      }

      return { successful: true, reason: '' };
    });
  }

  updateDimensions(width, height) {
    // Update width, height, and aspect ratio atomically
    const aspectRatio = width / height;
    const parameters = [
      new Parameter('width', ParameterType.PARAMETER_INTEGER, BigInt(width)),
      new Parameter('height', ParameterType.PARAMETER_INTEGER, BigInt(height)),
      new Parameter(
        'aspect_ratio',
        ParameterType.PARAMETER_DOUBLE,
        aspectRatio
      ),
    ];

    const result = this.node.setParametersAtomically(parameters);

    if (result.successful) {
      console.log(
        `Dimensions updated: ${width}x${height}, ratio: ${aspectRatio}`
      );
    } else {
      console.log(`Failed to update dimensions: ${result.reason}`);
    }

    return result;
  }
}
```

## Parameter Overrides

### Command Line Overrides

Override parameters when starting the node:

```javascript
// parameter-override-example.js
async function main() {
  const NODE_NAME = 'configurable_node';

  // Command line arguments for parameter override
  const argv = [
    '--ros-args',
    '-p',
    `${NODE_NAME}:robot_name:=autonomous_rover`,
    '-p',
    `${NODE_NAME}:max_speed:=3.5`,
    '-p',
    `${NODE_NAME}:debug_mode:=true`,
  ];

  // Initialize with command line arguments
  await rclnodejs.init(rclnodejs.Context.defaultContext(), argv);

  const node = rclnodejs.createNode(NODE_NAME);

  // Declare parameters with default values
  const parameters = [
    new Parameter(
      'robot_name',
      ParameterType.PARAMETER_STRING,
      'default_robot'
    ),
    new Parameter('max_speed', ParameterType.PARAMETER_DOUBLE, 1.0),
    new Parameter('debug_mode', ParameterType.PARAMETER_BOOL, false),
  ];

  node.declareParameters(parameters);

  // Values will be overridden by command line arguments
  parameters.forEach((param) => {
    const current = node.getParameter(param.name);
    console.log(
      `${param.name}: ${current.value} (overridden: ${current.value !== param.value})`
    );
  });

  rclnodejs.shutdown();
}

main().catch(console.error);
```

Run with overrides:

```bash
node parameter-override-example.js --ros-args -p configurable_node:robot_name:=my_robot
```

### Automatic Parameter Declaration

```javascript
const NodeOptions = rclnodejs.NodeOptions;

async function autoParametersExample() {
  await rclnodejs.init();

  // Enable automatic parameter declaration from overrides
  const options = new NodeOptions();
  options.automaticallyDeclareParametersFromOverrides = true;

  const node = rclnodejs.createNode('auto_param_node', options);

  // Any command line parameters will be automatically declared
  const allParams = node.getParameterNames();
  console.log(`Auto-declared parameters: ${allParams.join(', ')}`);

  // Get parameter overrides that were applied
  const overrides = node.getParameterOverrides();
  console.log('Parameter overrides:', overrides);

  rclnodejs.shutdown();
}
```

## Advanced Features

### Parameter Namespaces

```javascript
class NamespacedParameterNode {
  constructor() {
    this.node = rclnodejs.createNode('namespaced_node');
    this.setupNamespacedParameters();
  }

  setupNamespacedParameters() {
    // Create hierarchical parameter structure using dot notation
    const parameters = [
      // Camera parameters namespace
      new Parameter('camera.width', ParameterType.PARAMETER_INTEGER, 640n),
      new Parameter('camera.height', ParameterType.PARAMETER_INTEGER, 480n),
      new Parameter('camera.fps', ParameterType.PARAMETER_DOUBLE, 30.0),

      // Sensor parameters namespace
      new Parameter('sensor.imu.rate', ParameterType.PARAMETER_DOUBLE, 100.0),
      new Parameter('sensor.lidar.range', ParameterType.PARAMETER_DOUBLE, 10.0),

      // Control parameters namespace
      new Parameter('control.pid.kp', ParameterType.PARAMETER_DOUBLE, 1.0),
      new Parameter('control.pid.ki', ParameterType.PARAMETER_DOUBLE, 0.1),
      new Parameter('control.pid.kd', ParameterType.PARAMETER_DOUBLE, 0.01),
    ];

    this.node.declareParameters(parameters);
  }

  getCameraConfig() {
    const cameraParams = this.node.getParameters([
      'camera.width',
      'camera.height',
      'camera.fps',
    ]);

    return {
      width: Number(cameraParams[0].value),
      height: Number(cameraParams[1].value),
      fps: cameraParams[2].value,
    };
  }

  getPIDConfig() {
    const pidParams = this.node.getParameters([
      'control.pid.kp',
      'control.pid.ki',
      'control.pid.kd',
    ]);

    return {
      kp: pidParams[0].value,
      ki: pidParams[1].value,
      kd: pidParams[2].value,
    };
  }

  // Update all PID parameters atomically
  updatePIDGains(kp, ki, kd) {
    const pidParams = [
      new Parameter('control.pid.kp', ParameterType.PARAMETER_DOUBLE, kp),
      new Parameter('control.pid.ki', ParameterType.PARAMETER_DOUBLE, ki),
      new Parameter('control.pid.kd', ParameterType.PARAMETER_DOUBLE, kd),
    ];

    const result = this.node.setParametersAtomically(pidParams);

    if (result.successful) {
      console.log(`PID gains updated: kp=${kp}, ki=${ki}, kd=${kd}`);
    } else {
      console.error(`Failed to update PID gains: ${result.reason}`);
    }

    return result;
  }
}
```

### Parameter Addressing

Parameters are addressed using a hierarchical system:

```
/node_namespace/node_name:parameter_namespace.parameter_name
```

- **Node namespace**: Optional namespace for the node (e.g., `/robot1/`)
- **Node name**: The name of the node containing the parameter
- **Parameter namespace**: Optional namespace within the parameter name (using `.` separator)
- **Parameter name**: The actual parameter identifier

Example addressing:

```bash
# Global parameter
/my_node:max_speed

# Namespaced node parameter
/robot1/my_node:max_speed

# Nested parameter namespace
/my_node:camera.fps

# Fully qualified parameter
/robot1/sensors/camera_node:config.exposure.auto
```

### Parameter Events and Monitoring

```javascript
class ParameterMonitorNode {
  constructor() {
    this.node = rclnodejs.createNode('param_monitor_node');
    this.setupParameters();
    this.setupParameterMonitoring();
  }

  setupParameters() {
    const params = [
      new Parameter(
        'temperature_threshold',
        ParameterType.PARAMETER_DOUBLE,
        75.0
      ),
      new Parameter('emergency_stop', ParameterType.PARAMETER_BOOL, false),
      new Parameter('log_level', ParameterType.PARAMETER_STRING, 'INFO'),
    ];

    this.node.declareParameters(params);
  }

  setupParameterMonitoring() {
    // Monitor parameter changes with callback
    this.node.addOnSetParametersCallback((parameters) => {
      parameters.forEach((param) => {
        this.handleParameterChange(param);
      });

      return { successful: true, reason: '' };
    });
  }

  handleParameterChange(parameter) {
    console.log(`Parameter changed: ${parameter.name} = ${parameter.value}`);

    switch (parameter.name) {
      case 'temperature_threshold':
        if (parameter.value > 100.0) {
          console.warn('High temperature threshold set!');
        }
        break;

      case 'emergency_stop':
        if (parameter.value === true) {
          console.error('EMERGENCY STOP ACTIVATED!');
          this.emergencyShutdown();
        }
        break;

      case 'log_level':
        console.log(`Log level changed to: ${parameter.value}`);
        this.updateLogLevel(parameter.value);
        break;
    }
  }

  emergencyShutdown() {
    console.log('Initiating emergency shutdown sequence...');
    // Implement emergency shutdown logic
  }

  updateLogLevel(level) {
    // Update logging configuration
    console.log(`Logging level updated to: ${level}`);
  }
}
```

## Best Practices

### 1. Parameter Validation

Always validate parameters to ensure system safety:

```javascript
class SafeParameterNode {
  constructor() {
    this.node = rclnodejs.createNode('safe_param_node');
    this.setupValidatedParameters();
  }

  setupValidatedParameters() {
    // Declare parameters with proper descriptors
    const speedParam = new Parameter(
      'max_speed',
      ParameterType.PARAMETER_DOUBLE,
      1.0
    );
    const speedDescriptor = new ParameterDescriptor(
      'max_speed',
      ParameterType.PARAMETER_DOUBLE,
      'Maximum robot speed in m/s'
    );

    // Add range validation
    const FloatingPointRange = rclnodejs.FloatingPointRange;
    speedDescriptor.range = new FloatingPointRange(0.0, 10.0, 0.1);

    this.node.declareParameter(speedParam, speedDescriptor);

    // Add comprehensive validation
    this.node.addOnSetParametersCallback((parameters) => {
      for (const param of parameters) {
        const validation = this.validateParameter(param);
        if (!validation.valid) {
          return {
            successful: false,
            reason: validation.reason,
          };
        }
      }
      return { successful: true, reason: '' };
    });
  }

  validateParameter(parameter) {
    switch (parameter.name) {
      case 'max_speed':
        if (parameter.value < 0) {
          return { valid: false, reason: 'Speed cannot be negative' };
        }
        if (parameter.value > 5.0) {
          return { valid: false, reason: 'Speed too high for safety limits' };
        }
        break;
    }
    return { valid: true, reason: '' };
  }
}
```

### 2. Parameter Documentation

```javascript
// Well-documented parameter setup
class DocumentedParameterNode {
  constructor() {
    this.node = rclnodejs.createNode('documented_node');
    this.declareWellDocumentedParameters();
  }

  declareWellDocumentedParameters() {
    const parameters = [
      {
        param: new Parameter(
          'update_frequency',
          ParameterType.PARAMETER_DOUBLE,
          10.0
        ),
        descriptor: new ParameterDescriptor(
          'update_frequency',
          ParameterType.PARAMETER_DOUBLE,
          'Rate at which sensor data is processed (Hz). Valid range: 1-100 Hz.'
        ),
      },
      {
        param: new Parameter(
          'sensor_timeout',
          ParameterType.PARAMETER_DOUBLE,
          5.0
        ),
        descriptor: new ParameterDescriptor(
          'sensor_timeout',
          ParameterType.PARAMETER_DOUBLE,
          'Maximum time to wait for sensor data before timeout (seconds).'
        ),
      },
    ];

    parameters.forEach(({ param, descriptor }) => {
      this.node.declareParameter(param, descriptor);
    });
  }
}
```

### 3. Error Handling

```javascript
class RobustParameterNode {
  constructor() {
    this.node = rclnodejs.createNode('robust_param_node');
    this.safeParameterOperations();
  }

  safeParameterOperations() {
    try {
      // Safe parameter declaration
      const param = new Parameter(
        'critical_value',
        ParameterType.PARAMETER_DOUBLE,
        1.0
      );
      this.node.declareParameter(param);

      // Safe parameter access
      if (this.node.hasParameter('critical_value')) {
        const value = this.node.getParameter('critical_value');
        console.log(`Critical value: ${value.value}`);
      } else {
        console.warn('Critical parameter not found, using default');
      }
    } catch (error) {
      console.error(`Parameter operation failed: ${error.message}`);
      // Implement fallback behavior
      this.useDefaultConfiguration();
    }
  }

  useDefaultConfiguration() {
    console.log('Using default configuration due to parameter errors');
    // Implement safe defaults
  }

  safeParameterUpdate(name, value, type) {
    try {
      if (!this.node.hasParameter(name)) {
        console.warn(`Parameter ${name} not declared, declaring now`);
        const param = new Parameter(name, type, value);
        this.node.declareParameter(param);
        return true;
      }

      const param = new Parameter(name, type, value);
      const result = this.node.setParameter(param);

      if (!result.successful) {
        console.error(`Failed to update ${name}: ${result.reason}`);
        return false;
      }

      return true;
    } catch (error) {
      console.error(`Parameter update error: ${error.message}`);
      return false;
    }
  }
}
```

## Running the Examples

The rclnodejs repository includes parameter examples in the `example/parameter/` directory.

### Run Parameter Declaration Example

```bash
# Terminal 1 - Run the parameter declaration example
cd /path/to/rclnodejs
node example/parameter/parameter-declaration-example.js
```

### Run Parameter Override Example

```bash
# Terminal 1 - Run with parameter overrides
cd /path/to/rclnodejs
node example/parameter/parameter-override-example.js --ros-args -p my_node:param1:=custom_value
```

### Using ROS 2 CLI Tools

You can interact with parameter services using ROS 2 command line tools:

```bash
# List all parameters for a node
ros2 param list /your_node_name

# List parameters with specific prefix
ros2 param list /your_node_name --filter camera

# Get parameter value
ros2 param get /your_node_name parameter_name

# Get multiple parameters
ros2 param get /your_node_name camera.width
ros2 param get /your_node_name camera.height

# Set parameter value
ros2 param set /your_node_name parameter_name new_value

# Set different parameter types
ros2 param set /your_node_name max_speed 2.5        # double
ros2 param set /your_node_name robot_name "rover"   # string
ros2 param set /your_node_name debug_mode true      # boolean
ros2 param set /your_node_name port_number 8080     # integer

# Set array parameters
ros2 param set /your_node_name topics "['topic1', 'topic2', 'topic3']"

# Describe parameter (get descriptor information)
ros2 param describe /your_node_name parameter_name

# Dump all parameters to YAML file
ros2 param dump /your_node_name > node_params.yaml

# Load parameters from YAML file
ros2 param load /your_node_name node_params.yaml

# Delete a parameter (if supported)
ros2 param delete /your_node_name parameter_name
```

### Parameter Monitoring

Monitor parameter changes in real-time:

```bash
# Watch parameter changes
ros2 param list /your_node_name --monitor

# Get parameter type information
ros2 param get-types /your_node_name
```

### Expected Output

**Parameter Declaration Example:**

```bash
Declared parameter: param1
Parameter details:  Parameter {
  name: 'param1',
  type: 4,
  value: 'hello world'
}
ParameterDescriptor {
  name: 'param1',
  type: 4,
  description: '',
  additionalConstraints: '',
  readOnly: false
}
```

**Parameter Override Example:**

```bash
Declared parameter: param1
Parameter overridden:  Parameter {
  name: 'param1',
  type: 4,
  value: 'hello ros2'
}
```
