# Type Description Service in rclnodejs

The Type Description Service is a built-in ROS 2 service that provides introspection capabilities for message and service types used by a node. This tutorial will guide you through understanding and using the Type Description Service with rclnodejs.

## Table of Contents

- [What is the Type Description Service?](#what-is-the-type-description-service)
- [Requirements](#requirements)
- [How Type Description Service Works](#how-type-description-service-works)
- [Automatic Service Creation](#automatic-service-creation)
- [Service Interface](#service-interface)
- [Basic Usage](#basic-usage)
- [Complete Examples](#complete-examples)
- [Type Hash Generation](#type-hash-generation)
- [Configuration and Parameters](#configuration-and-parameters)
- [Advanced Usage](#advanced-usage)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)
- [Code Pattern Summary](#code-pattern-summary)
- [Conclusion](#conclusion)

## What is the Type Description Service?

The Type Description Service is an automatic introspection service that provides detailed information about ROS 2 message and service types used by a node. It enables you to:

- **Discover type definitions** used by publishers, subscribers, services, and actions
- **Retrieve complete type information** including nested types and dependencies
- **Get type source code** and type relationships
- **Understand message structure** for debugging and development
- **Enable dynamic type discovery** for tools and debugging utilities

Each node automatically creates a service at:

```
/node_name/get_type_description
```

## Requirements

The Type Description Service is available in:

- **ROS 2 Iron** and newer distributions (officially supported starting from Iron)
- **rclnodejs** with compatible ROS 2 installation

**Note**: Type Description Service is **not available** in ROS 2 Humble and earlier versions.

```javascript
const rclnodejs = require('rclnodejs');
const DistroUtils = require('rclnodejs/lib/distro');

// Check if Type Description Service is supported
if (DistroUtils.getDistroId() > DistroUtils.getDistroId('humble')) {
  console.log('Type Description Service is supported');
} else {
  console.warn(
    'Type Description Service is not supported by this version of ROS 2'
  );
}
```

## How Type Description Service Works

When a node is created with Type Description Service enabled (default), it automatically:

1. **Monitors all publishers, subscribers, services, and actions** created by the node
2. **Tracks type information** for all message and service types used
3. **Creates a service** at `~/get_type_description` to respond to type queries
4. **Provides type definitions** including source code and dependencies

The service responds to requests with:

- **Type descriptions** including field names, types, and nested structures
- **Type sources** containing the original `.msg`, `.srv`, or `.action` definitions
- **Type relationships** showing dependencies between types

## Automatic Service Creation

The Type Description Service is automatically created when a node is initialized:

```javascript
const rclnodejs = require('rclnodejs');

async function createNodeWithTypeDescriptionService() {
  await rclnodejs.init();

  // Type Description Service is automatically enabled by default
  const node = rclnodejs.createNode('my_node');

  // The service is available at: /my_node/get_type_description
  console.log(
    'Type Description Service available at:',
    `/my_node/get_type_description`
  );

  rclnodejs.spin(node);
}
```

## Service Interface

The Type Description Service uses the `type_description_interfaces/srv/GetTypeDescription` interface:

### Request

```javascript
const request = {
  type_name: 'std_msgs/msg/String', // Full type name
  type_hash: 'RIHS01_abc123...', // Type hash string
  include_type_sources: true, // Whether to include source code
};
```

### Response

```javascript
const response = {
  successful: true, // Whether request was successful
  failure_reason: '', // Error message if unsuccessful
  type_description: {
    // Type structure information
    type_description: {
      type_name: 'std_msgs/msg/String',
      fields: [
        {
          name: 'data',
          type: {
            FIELD_TYPE_STRING: 4,
            string_upper_bound: 0,
            string_array_size: 0,
            array_size: 0,
            array_upper_bound: 0,
            nested_type_name: '',
          },
          default_value: '',
        },
      ],
    },
    referenced_type_descriptions: [], // Nested type definitions
  },
  type_sources: [
    // Source code definitions
    {
      type_name: 'std_msgs/msg/String',
      encoding: 'msg',
      raw_file_contents: 'string data\n',
    },
  ],
};
```

## Basic Usage

### Getting Type Information for a Publisher

```javascript
const rclnodejs = require('rclnodejs');
const TypeDescriptionService = require('rclnodejs/lib/type_description_service');

async function getPublisherTypeInfo() {
  await rclnodejs.init();

  const node = rclnodejs.createNode('type_info_example');

  // Create a publisher
  const publisher = node.createPublisher('std_msgs/msg/String', 'test_topic');

  // Get publisher information to extract type hash
  const publisherInfos = node.getPublishersInfoByTopic('/test_topic', false);
  console.log('Publisher info:', publisherInfos[0]);

  // Create client for the Type Description Service
  const client = node.createClient(
    'type_description_interfaces/srv/GetTypeDescription',
    '/type_info_example/get_type_description'
  );

  // Wait for service to be available
  if (!(await client.waitForService(5000))) {
    console.error('Type Description Service not available');
    return;
  }

  // Prepare request
  const request = {
    type_name: 'std_msgs/msg/String',
    type_hash: TypeDescriptionService.toTypeHash(
      publisherInfos[0].topic_type_hash
    ),
    include_type_sources: true,
  };

  // Start spinning
  rclnodejs.spin(node);

  // Send request
  client.sendRequest(request, (response) => {
    if (response.successful) {
      console.log('Type Description Retrieved Successfully:');
      console.log(
        'Type Name:',
        response.type_description.type_description.type_name
      );
      console.log('Fields:', response.type_description.type_description.fields);

      if (response.type_sources.length > 0) {
        console.log('Type Source:');
        console.log(response.type_sources[0].raw_file_contents);
      }
    } else {
      console.error('Failed to get type description:', response.failure_reason);
    }
  });
}

getPublisherTypeInfo().catch(console.error);
```

### Querying External Node's Type Information

```javascript
const rclnodejs = require('rclnodejs');

async function queryExternalNodeTypes() {
  await rclnodejs.init();

  const node = rclnodejs.createNode('type_query_client');

  // Query another node's type description service
  const targetNodeName = 'target_node';
  const serviceName = `/${targetNodeName}/get_type_description`;

  const client = node.createClient(
    'type_description_interfaces/srv/GetTypeDescription',
    serviceName
  );

  if (!(await client.waitForService(5000))) {
    console.error(
      `Type Description Service not available for node: ${targetNodeName}`
    );
    return;
  }

  // Request information about a specific type
  const request = {
    type_name: 'geometry_msgs/msg/Twist',
    type_hash: '', // Can be empty if you don't have the specific hash
    include_type_sources: true,
  };

  rclnodejs.spin(node);

  client.sendRequest(request, (response) => {
    if (response.successful) {
      console.log('External Node Type Information:');
      console.log(
        'Type:',
        response.type_description.type_description.type_name
      );

      // Print all fields
      response.type_description.type_description.fields.forEach((field) => {
        console.log(
          `Field: ${field.name}, Type: ${JSON.stringify(field.type)}`
        );
      });

      // Print source if available
      if (response.type_sources.length > 0) {
        console.log('Source Definition:');
        console.log(response.type_sources[0].raw_file_contents);
      }
    } else {
      console.error('Query failed:', response.failure_reason);
    }
  });
}
```

### Complete Test-Based Example

This example follows the exact patterns used in the official test suite:

```javascript
const rclnodejs = require('rclnodejs');
const TypeDescriptionService = require('rclnodejs/lib/type_description_service');
const DistroUtils = require('rclnodejs/lib/distro');

async function testTypeDescriptionService() {
  // Check if Type Description Service is supported
  if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
    console.log(
      'Type Description Service is not supported in this ROS 2 version'
    );
    return;
  }

  await rclnodejs.init();

  const nodeName = 'test_type_description_service';
  const node = rclnodejs.createNode(nodeName);
  rclnodejs.spin(node);

  // Create a publisher (this registers the type with the node)
  const topic = 'test_get_type_description_publisher';
  const topicType = 'std_msgs/msg/String';
  node.createPublisher(topicType, topic);

  // Get publisher information to extract type hash
  const infos = node.getPublishersInfoByTopic(
    '/test_get_type_description_publisher',
    false
  );

  if (infos.length === 0) {
    throw new Error('Publisher info not found');
  }

  console.log('Publisher info found:', infos[0]);

  // Create a client to get the type description
  const request = {
    type_name: topicType,
    type_hash: TypeDescriptionService.toTypeHash(infos[0].topic_type_hash),
    include_type_sources: true,
  };

  const serviceName = `/test_type_description_service/get_type_description`;
  const GetTypeDescription =
    'type_description_interfaces/srv/GetTypeDescription';
  const client = node.createClient(GetTypeDescription, serviceName);

  const result = await client.waitForService(5000);
  if (!result) {
    throw new Error('Service not available');
  }

  // Send request and handle response
  const promise = new Promise((resolve, reject) => {
    const timer = setInterval(() => {
      client.sendRequest(request, (response) => {
        clearInterval(timer);

        console.log('Response received:');
        console.log('- Successful:', response.successful);

        if (response.successful) {
          console.log(
            '- Type name:',
            response.type_description.type_description.type_name
          );
          console.log(
            '- Number of fields:',
            response.type_description.type_description.fields.length
          );
          console.log('- Type sources length:', response.type_sources.length);

          // Print field information
          response.type_description.type_description.fields.forEach(
            (field, index) => {
              console.log(
                `  Field ${index}: ${field.name} (${JSON.stringify(field.type)})`
              );
            }
          );

          // Print source if available
          if (response.type_sources.length > 0) {
            console.log('- Raw file contents:');
            console.log(response.type_sources[0].raw_file_contents);
          }

          resolve(response);
        } else {
          reject(
            new Error(
              `Type description request failed: ${response.failure_reason}`
            )
          );
        }
      });
    }, 2000);
  });

  await promise;
  console.log('Test completed successfully');
  rclnodejs.shutdown();
}

// Run the test
testTypeDescriptionService().catch(console.error);
```

### Parameter Configuration Test

The Type Description Service can be configured via parameters. Here's how to check the configuration:

```javascript
const { exec } = require('child_process');
const rclnodejs = require('rclnodejs');

async function testParameterConfiguration() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('test_type_description_service');
  rclnodejs.spin(node);

  // Check if start_type_description_service parameter exists
  const checkParamExists = () => {
    return new Promise((resolve, reject) => {
      exec(
        'ros2 param list /test_type_description_service',
        (error, stdout, stderr) => {
          if (error || stderr) {
            reject(new Error('Failed to list parameters'));
            return;
          }

          if (stdout.includes('start_type_description_service')) {
            console.log('✓ start_type_description_service parameter found');
            resolve(true);
          } else {
            reject(
              new Error("'start_type_description_service' parameter not found")
            );
          }
        }
      );
    });
  };

  // Check parameter value
  const checkParamValue = () => {
    return new Promise((resolve, reject) => {
      exec(
        'ros2 param get /test_type_description_service start_type_description_service',
        (error, stdout, stderr) => {
          if (error || stderr) {
            reject(new Error('Failed to get parameter value'));
            return;
          }

          if (stdout.includes('Boolean value is: True')) {
            console.log('✓ start_type_description_service is enabled (True)');
            resolve(true);
          } else {
            console.log('Parameter value output:', stdout);
            reject(new Error("Parameter value is not 'True'"));
          }
        }
      );
    });
  };

  try {
    await checkParamExists();
    await checkParamValue();
    console.log('All parameter configuration tests passed');
  } catch (error) {
    console.error('Parameter configuration test failed:', error.message);
  } finally {
    rclnodejs.shutdown();
  }
}

// Run parameter test
testParameterConfiguration().catch(console.error);
```

## Complete Examples

### Type Discovery Tool

```javascript
const rclnodejs = require('rclnodejs');
const TypeDescriptionService = require('rclnodejs/lib/type_description_service');

class TypeDiscoveryTool {
  constructor() {
    this.node = null;
    this.discoveredTypes = new Map();
  }

  async start() {
    await rclnodejs.init();
    this.node = rclnodejs.createNode('type_discovery_tool');
    rclnodejs.spin(this.node);
    console.log('Type Discovery Tool started');
  }

  async discoverNodeTypes(nodeName) {
    console.log(`\\nDiscovering types for node: ${nodeName}`);

    const serviceName = `/${nodeName}/get_type_description`;
    const client = this.node.createClient(
      'type_description_interfaces/srv/GetTypeDescription',
      serviceName
    );

    if (!(await client.waitForService(2000))) {
      console.error(
        `Type Description Service not available for node: ${nodeName}`
      );
      return;
    }

    // Get all topics and their types
    const topicNamesAndTypes = this.node.getTopicNamesAndTypes();
    const serviceNamesAndTypes = this.node.getServiceNamesAndTypes();

    const typesToQuery = new Set();

    // Collect message types from topics
    topicNamesAndTypes.forEach(([topicName, types]) => {
      types.forEach((type) => typesToQuery.add(type));
    });

    // Collect service types
    serviceNamesAndTypes.forEach(([serviceName, types]) => {
      types.forEach((type) => typesToQuery.add(type));
    });

    // Query each unique type
    for (const typeName of typesToQuery) {
      await this.queryType(client, typeName);
    }
  }

  async queryType(client, typeName) {
    return new Promise((resolve) => {
      const request = {
        type_name: typeName,
        type_hash: '', // Empty hash - let service handle it
        include_type_sources: true,
      };

      client.sendRequest(request, (response) => {
        if (response.successful) {
          this.processTypeDescription(typeName, response);
        } else {
          console.log(
            `Failed to get type description for ${typeName}: ${response.failure_reason}`
          );
        }
        resolve();
      });
    });
  }

  processTypeDescription(typeName, response) {
    const typeDesc = response.type_description.type_description;

    console.log(`\\n=== Type: ${typeName} ===`);
    console.log(`Fields (${typeDesc.fields.length}):`);

    typeDesc.fields.forEach((field) => {
      const typeInfo = this.formatFieldType(field.type);
      console.log(`  ${field.name}: ${typeInfo}`);
    });

    // Store for later reference
    this.discoveredTypes.set(typeName, {
      description: typeDesc,
      sources: response.type_sources,
    });

    // Print source if available
    if (response.type_sources.length > 0) {
      console.log(`Source (${response.type_sources[0].encoding}):`);
      console.log(response.type_sources[0].raw_file_contents.trim());
    }
  }

  formatFieldType(fieldType) {
    // Map field type constants to readable names
    const typeMap = {
      1: 'bool',
      2: 'byte',
      3: 'char',
      4: 'string',
      5: 'wstring',
      6: 'float32',
      7: 'float64',
      8: 'int8',
      9: 'uint8',
      10: 'int16',
      11: 'uint16',
      12: 'int32',
      13: 'uint32',
      14: 'int64',
      15: 'uint64',
      16: 'nested_type',
    };

    for (const [key, value] of Object.entries(fieldType)) {
      if (key.startsWith('FIELD_TYPE_') && value > 0) {
        const typeNum = value;
        let typeName = typeMap[typeNum] || `unknown(${typeNum})`;

        if (typeName === 'nested_type' && fieldType.nested_type_name) {
          typeName = fieldType.nested_type_name;
        }

        // Handle arrays
        if (fieldType.array_size > 0) {
          typeName += `[${fieldType.array_size}]`;
        } else if (fieldType.array_upper_bound > 0) {
          typeName += `[<=${fieldType.array_upper_bound}]`;
        }

        return typeName;
      }
    }

    return 'unknown';
  }

  printSummary() {
    console.log(`\\n=== Discovery Summary ===`);
    console.log(`Discovered ${this.discoveredTypes.size} types:`);

    Array.from(this.discoveredTypes.keys())
      .sort()
      .forEach((typeName) => {
        const typeInfo = this.discoveredTypes.get(typeName);
        console.log(
          `  ${typeName} (${typeInfo.description.fields.length} fields)`
        );
      });
  }
}

// Usage
async function runTypeDiscovery() {
  const tool = new TypeDiscoveryTool();
  await tool.start();

  // Discover types for specific nodes
  await tool.discoverNodeTypes('my_node');

  tool.printSummary();
}

runTypeDiscovery().catch(console.error);
```

### Automatic Type Documentation Generator

```javascript
const rclnodejs = require('rclnodejs');
const fs = require('fs');

class TypeDocumentationGenerator {
  constructor() {
    this.node = null;
    this.documentation = [];
  }

  async start() {
    await rclnodejs.init();
    this.node = rclnodejs.createNode('type_doc_generator');
    rclnodejs.spin(this.node);
  }

  async generateDocumentation(nodeName, outputFile) {
    console.log(`Generating documentation for node: ${nodeName}`);

    const serviceName = `/${nodeName}/get_type_description`;
    const client = this.node.createClient(
      'type_description_interfaces/srv/GetTypeDescription',
      serviceName
    );

    if (!(await client.waitForService(5000))) {
      throw new Error(
        `Type Description Service not available for node: ${nodeName}`
      );
    }

    // Get all topics for this node
    const topics = this.node.getTopicNamesAndTypes();
    const nodeTopics = topics.filter(
      ([topicName]) =>
        topicName.startsWith(`/${nodeName}/`) || topicName === `/${nodeName}`
    );

    for (const [topicName, types] of nodeTopics) {
      for (const typeName of types) {
        await this.documentType(client, typeName, topicName);
      }
    }

    // Generate markdown documentation
    const markdown = this.generateMarkdown(nodeName);
    fs.writeFileSync(outputFile, markdown);
    console.log(`Documentation written to: ${outputFile}`);
  }

  async documentType(client, typeName, topicName) {
    return new Promise((resolve) => {
      const request = {
        type_name: typeName,
        type_hash: '',
        include_type_sources: true,
      };

      client.sendRequest(request, (response) => {
        if (response.successful) {
          this.documentation.push({
            typeName,
            topicName,
            description: response.type_description.type_description,
            sources: response.type_sources,
          });
        }
        resolve();
      });
    });
  }

  generateMarkdown(nodeName) {
    let markdown = `# Type Documentation for Node: ${nodeName}\\n\\n`;
    markdown += `Generated on: ${new Date().toISOString()}\\n\\n`;

    this.documentation.forEach((doc) => {
      markdown += `## ${doc.typeName}\\n\\n`;
      markdown += `**Topic**: \`${doc.topicName}\`\\n\\n`;

      if (doc.description.fields.length > 0) {
        markdown += `### Fields\\n\\n`;
        markdown += `| Field Name | Type | Default Value |\\n`;
        markdown += `|------------|------|---------------|\\n`;

        doc.description.fields.forEach((field) => {
          const typeStr = this.formatFieldTypeForDoc(field.type);
          markdown += `| \`${field.name}\` | ${typeStr} | \`${field.default_value || 'N/A'}\` |\\n`;
        });
        markdown += `\\n`;
      }

      if (doc.sources.length > 0) {
        markdown += `### Source Definition\\n\\n`;
        markdown += `\`\`\`${doc.sources[0].encoding}\\n`;
        markdown += doc.sources[0].raw_file_contents;
        markdown += `\`\`\`\\n\\n`;
      }

      markdown += `---\\n\\n`;
    });

    return markdown;
  }

  formatFieldTypeForDoc(fieldType) {
    // Similar to previous formatFieldType but with markdown formatting
    const typeMap = {
      1: 'bool',
      2: 'byte',
      3: 'char',
      4: 'string',
      5: 'wstring',
      6: 'float32',
      7: 'float64',
      8: 'int8',
      9: 'uint8',
      10: 'int16',
      11: 'uint16',
      12: 'int32',
      13: 'uint32',
      14: 'int64',
      15: 'uint64',
      16: 'nested_type',
    };

    for (const [key, value] of Object.entries(fieldType)) {
      if (key.startsWith('FIELD_TYPE_') && value > 0) {
        let typeName = typeMap[value] || `unknown(${value})`;

        if (typeName === 'nested_type' && fieldType.nested_type_name) {
          typeName = `\`${fieldType.nested_type_name}\``;
        } else {
          typeName = `\`${typeName}\``;
        }

        if (fieldType.array_size > 0) {
          typeName += `[${fieldType.array_size}]`;
        } else if (fieldType.array_upper_bound > 0) {
          typeName += `[≤${fieldType.array_upper_bound}]`;
        }

        return typeName;
      }
    }

    return '`unknown`';
  }
}

// Usage
async function generateDocumentation() {
  const generator = new TypeDocumentationGenerator();
  await generator.start();

  await generator.generateDocumentation('my_robot_node', 'robot_types.md');
}
```

## Type Hash Generation

The Type Description Service uses type hashes to uniquely identify message structures:

```javascript
const TypeDescriptionService = require('rclnodejs/lib/type_description_service');

// Convert topic type hash to string format
function demonstrateTypeHashGeneration(node) {
  // Create a publisher to get type hash
  const publisher = node.createPublisher('geometry_msgs/msg/Twist', 'cmd_vel');

  // Get publisher info with type hash
  const publisherInfos = node.getPublishersInfoByTopic('/cmd_vel', false);

  if (publisherInfos.length > 0) {
    const topicTypeHash = publisherInfos[0].topic_type_hash;

    console.log('Raw type hash:', topicTypeHash);
    console.log('Version:', topicTypeHash.version);
    console.log('Value (hex):', topicTypeHash.value.toString('hex'));

    // Convert to string format used by Type Description Service
    const hashString = TypeDescriptionService.toTypeHash(topicTypeHash);
    console.log('Hash string:', hashString);
    // Output: RIHS01_abcd1234ef567890...
  }
}
```

## Configuration and Parameters

### Controlling Type Description Service

The Type Description Service can be controlled via node parameters:

```javascript
const rclnodejs = require('rclnodejs');

async function configureTypeDescriptionService() {
  await rclnodejs.init();

  // Method 1: Control via node options
  const nodeOptions = new rclnodejs.NodeOptions();
  nodeOptions.startTypeDescriptionService = false; // Disable the service

  const node = rclnodejs.createNode('configured_node', nodeOptions);

  // Method 2: Check parameter value
  const paramName = 'start_type_description_service';
  if (node.hasParameter(paramName)) {
    const param = node.getParameter(paramName);
    console.log('Type Description Service enabled:', param.value);
  }

  rclnodejs.spin(node);
}
```

### Runtime Parameter Management

```javascript
// Check if Type Description Service is running
function checkTypeDescriptionServiceStatus(node) {
  const paramName = 'start_type_description_service';

  if (node.hasParameter(paramName)) {
    const param = node.getParameter(paramName);
    console.log(
      'Type Description Service status:',
      param.value ? 'ENABLED' : 'DISABLED'
    );

    // The service name would be
    const serviceName = `/${node.name()}/get_type_description`;
    console.log('Service available at:', serviceName);

    return param.value;
  }

  return false;
}

// List all Type Description Services in the system
async function listAllTypeDescriptionServices() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('service_lister');

  const services = node.getServiceNamesAndTypes();
  const typeDescServices = services.filter(([serviceName, types]) =>
    serviceName.endsWith('/get_type_description')
  );

  console.log('Available Type Description Services:');
  typeDescServices.forEach(([serviceName, types]) => {
    const nodeName = serviceName.replace('/get_type_description', '');
    console.log(`  Node: ${nodeName}`);
    console.log(`  Service: ${serviceName}`);
    console.log(`  Types: ${types.join(', ')}`);
    console.log();
  });

  rclnodejs.shutdown();
}
```

## Advanced Usage

### Recursive Type Analysis

```javascript
class RecursiveTypeAnalyzer {
  constructor(node) {
    this.node = node;
    this.analyzedTypes = new Set();
    this.typeHierarchy = new Map();
  }

  async analyzeTypeRecursively(nodeName, rootTypeName) {
    const serviceName = `/${nodeName}/get_type_description`;
    const client = this.node.createClient(
      'type_description_interfaces/srv/GetTypeDescription',
      serviceName
    );

    if (!(await client.waitForService(5000))) {
      throw new Error(`Service not available: ${serviceName}`);
    }

    await this.analyzeType(client, rootTypeName, 0);
    return this.typeHierarchy;
  }

  async analyzeType(client, typeName, depth) {
    if (this.analyzedTypes.has(typeName)) {
      return; // Already analyzed
    }

    const indent = '  '.repeat(depth);
    console.log(`${indent}Analyzing: ${typeName}`);

    const response = await this.queryType(client, typeName);
    if (!response.successful) {
      console.log(`${indent}Failed: ${response.failure_reason}`);
      return;
    }

    this.analyzedTypes.add(typeName);
    const typeDesc = response.type_description.type_description;

    const typeInfo = {
      name: typeName,
      fields: [],
      nestedTypes: [],
    };

    // Process each field
    for (const field of typeDesc.fields) {
      const fieldInfo = {
        name: field.name,
        type: this.extractFieldType(field.type),
        defaultValue: field.default_value,
      };

      typeInfo.fields.push(fieldInfo);

      // If it's a nested type, analyze it recursively
      if (field.type.nested_type_name) {
        typeInfo.nestedTypes.push(field.type.nested_type_name);
        await this.analyzeType(client, field.type.nested_type_name, depth + 1);
      }
    }

    this.typeHierarchy.set(typeName, typeInfo);
  }

  async queryType(client, typeName) {
    return new Promise((resolve) => {
      const request = {
        type_name: typeName,
        type_hash: '',
        include_type_sources: true,
      };

      client.sendRequest(request, (response) => {
        resolve(response);
      });
    });
  }

  extractFieldType(fieldType) {
    // Extract the actual type from the field type structure
    for (const [key, value] of Object.entries(fieldType)) {
      if (key.startsWith('FIELD_TYPE_') && value > 0) {
        if (fieldType.nested_type_name) {
          return fieldType.nested_type_name;
        }
        return key.replace('FIELD_TYPE_', '').toLowerCase();
      }
    }
    return 'unknown';
  }

  printHierarchy() {
    console.log('\\n=== Type Hierarchy ===');
    this.typeHierarchy.forEach((typeInfo, typeName) => {
      console.log(`\\n${typeName}:`);
      typeInfo.fields.forEach((field) => {
        console.log(`  ${field.name}: ${field.type}`);
      });

      if (typeInfo.nestedTypes.length > 0) {
        console.log(`  Nested types: ${typeInfo.nestedTypes.join(', ')}`);
      }
    });
  }
}
```

### Performance Monitoring

```javascript
class TypeServicePerformanceMonitor {
  constructor() {
    this.queryTimes = [];
    this.successCount = 0;
    this.failureCount = 0;
  }

  async benchmarkTypeQueries(nodeName, typeNames, iterations = 10) {
    await rclnodejs.init();
    const node = rclnodejs.createNode('performance_monitor');

    const serviceName = `/${nodeName}/get_type_description`;
    const client = node.createClient(
      'type_description_interfaces/srv/GetTypeDescription',
      serviceName
    );

    if (!(await client.waitForService(5000))) {
      throw new Error(`Service not available: ${serviceName}`);
    }

    rclnodejs.spin(node);

    console.log(
      `Benchmarking ${typeNames.length} types with ${iterations} iterations each...`
    );

    for (const typeName of typeNames) {
      for (let i = 0; i < iterations; i++) {
        await this.timeQuery(client, typeName);
      }
    }

    this.printResults();
  }

  async timeQuery(client, typeName) {
    const startTime = process.hrtime.bigint();

    return new Promise((resolve) => {
      const request = {
        type_name: typeName,
        type_hash: '',
        include_type_sources: true,
      };

      client.sendRequest(request, (response) => {
        const endTime = process.hrtime.bigint();
        const durationMs = Number(endTime - startTime) / 1000000;

        this.queryTimes.push(durationMs);

        if (response.successful) {
          this.successCount++;
        } else {
          this.failureCount++;
        }

        resolve();
      });
    });
  }

  printResults() {
    const totalQueries = this.queryTimes.length;
    const avgTime = this.queryTimes.reduce((a, b) => a + b, 0) / totalQueries;
    const minTime = Math.min(...this.queryTimes);
    const maxTime = Math.max(...this.queryTimes);

    console.log('\\n=== Performance Results ===');
    console.log(`Total queries: ${totalQueries}`);
    console.log(`Successful: ${this.successCount}`);
    console.log(`Failed: ${this.failureCount}`);
    console.log(`Average time: ${avgTime.toFixed(2)} ms`);
    console.log(`Min time: ${minTime.toFixed(2)} ms`);
    console.log(`Max time: ${maxTime.toFixed(2)} ms`);
    console.log(
      `Success rate: ${((this.successCount / totalQueries) * 100).toFixed(1)}%`
    );
  }
}
```

## Best Practices

### 1. Handle Service Availability

```javascript
async function robustTypeQuery(nodeName, typeName) {
  const maxRetries = 3;
  const retryDelay = 1000; // ms

  for (let attempt = 1; attempt <= maxRetries; attempt++) {
    try {
      const client = node.createClient(
        'type_description_interfaces/srv/GetTypeDescription',
        `/${nodeName}/get_type_description`
      );

      if (await client.waitForService(2000)) {
        return await queryTypeWithTimeout(client, typeName, 5000);
      } else {
        console.log(`Attempt ${attempt}: Service not available`);
      }
    } catch (error) {
      console.log(`Attempt ${attempt} failed:`, error.message);
    }

    if (attempt < maxRetries) {
      await new Promise((resolve) => setTimeout(resolve, retryDelay));
    }
  }

  throw new Error(`Failed to query type after ${maxRetries} attempts`);
}

function queryTypeWithTimeout(client, typeName, timeoutMs) {
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      reject(new Error('Query timeout'));
    }, timeoutMs);

    const request = {
      type_name: typeName,
      type_hash: '',
      include_type_sources: true,
    };

    client.sendRequest(request, (response) => {
      clearTimeout(timeout);
      resolve(response);
    });
  });
}
```

### 2. Cache Type Information

```javascript
class TypeInformationCache {
  constructor() {
    this.cache = new Map();
    this.cacheTimeout = 300000; // 5 minutes
  }

  async getTypeDescription(nodeName, typeName) {
    const cacheKey = `${nodeName}:${typeName}`;
    const cached = this.cache.get(cacheKey);

    if (cached && Date.now() - cached.timestamp < this.cacheTimeout) {
      console.log(`Using cached type info for ${typeName}`);
      return cached.data;
    }

    // Fetch fresh data
    const typeInfo = await this.fetchTypeDescription(nodeName, typeName);

    // Cache it
    this.cache.set(cacheKey, {
      data: typeInfo,
      timestamp: Date.now(),
    });

    return typeInfo;
  }

  async fetchTypeDescription(nodeName, typeName) {
    // Implementation similar to previous examples
    // ... fetch logic here
  }

  clearCache() {
    this.cache.clear();
  }

  pruneExpiredEntries() {
    const now = Date.now();
    for (const [key, value] of this.cache.entries()) {
      if (now - value.timestamp >= this.cacheTimeout) {
        this.cache.delete(key);
      }
    }
  }
}
```

### 3. Version Compatibility

```javascript
const DistroUtils = require('rclnodejs/lib/distro');

function checkTypeDescriptionServiceSupport() {
  const distroId = DistroUtils.getDistroId();
  const jazzyId = DistroUtils.getDistroId('jazzy');

  if (distroId < jazzyId) {
    console.warn('Type Description Service requires ROS 2 Jazzy or newer');
    console.warn(`Current distribution ID: ${distroId}, Required: ${jazzyId}`);
    return false;
  }

  return true;
}

// Graceful degradation
async function createNodeWithOptionalTypeService(nodeName) {
  const options = new rclnodejs.NodeOptions();

  if (checkTypeDescriptionServiceSupport()) {
    options.startTypeDescriptionService = true;
    console.log('Type Description Service enabled');
  } else {
    options.startTypeDescriptionService = false;
    console.log('Type Description Service disabled (unsupported)');
  }

  return rclnodejs.createNode(nodeName, options);
}
```

## Troubleshooting

### Common Issues

#### 1. "Service not available" errors

**Problem**: Cannot connect to Type Description Service.

**Solutions**:

```bash
# Check if service exists
ros2 service list | grep get_type_description

# Check if node is running
ros2 node list

# Test service manually
ros2 service call /node_name/get_type_description type_description_interfaces/srv/GetTypeDescription "type_name: 'std_msgs/msg/String'
type_hash: ''
include_type_sources: true"
```

#### 2. Empty or invalid responses

**Problem**: Service responds but with no useful data.

**Solution**: Check the request format and ensure type name is correct:

```javascript
// Correct format
const request = {
  type_name: 'std_msgs/msg/String', // Full interface name
  type_hash: '', // Can be empty
  include_type_sources: true, // Set to true for source code
};

// Common mistakes:
// type_name: 'String'              // ❌ Missing package
// type_name: 'std_msgs/String'     // ❌ Missing msg/srv/action
```

#### 3. "Type not found" responses

**Problem**: Requested type is not known to the node.

**Solution**: Ensure the node actually uses the requested type:

```javascript
// Check what types the node actually uses
const topics = node.getTopicNamesAndTypes();
const services = node.getServiceNamesAndTypes();

console.log('Available message types:', [
  ...new Set(topics.flatMap(([_, types]) => types)),
]);
console.log('Available service types:', [
  ...new Set(services.flatMap(([_, types]) => types)),
]);
```

#### 4. Service disabled by parameter

**Problem**: Type Description Service is disabled via parameter.

**Solution**: Check and modify the parameter:

```bash
# Check parameter value
ros2 param get /node_name start_type_description_service

# Enable the service (requires node restart)
ros2 param set /node_name start_type_description_service true
```

### Debugging Tips

#### 1. Enable verbose logging

```javascript
// Add detailed logging to your type queries
function debugTypeQuery(client, typeName) {
  console.log(`Querying type: ${typeName}`);

  const request = {
    type_name: typeName,
    type_hash: '',
    include_type_sources: true,
  };

  console.log('Request:', JSON.stringify(request, null, 2));

  client.sendRequest(request, (response) => {
    console.log('Response received');
    console.log('Successful:', response.successful);

    if (!response.successful) {
      console.error('Failure reason:', response.failure_reason);
    } else {
      console.log(
        'Type name:',
        response.type_description.type_description.type_name
      );
      console.log(
        'Field count:',
        response.type_description.type_description.fields.length
      );
      console.log('Source count:', response.type_sources.length);
    }
  });
}
```

#### 2. Validate service interface

```bash
# Check service interface details
ros2 service info /node_name/get_type_description

# Test with a known type
ros2 service call /node_name/get_type_description type_description_interfaces/srv/GetTypeDescription "{type_name: 'std_msgs/msg/String', type_hash: '', include_type_sources: true}"
```

## Code Pattern Summary

Based on the official test implementation, use these patterns for reliable Type Description Service usage:

### Required Imports

```javascript
const rclnodejs = require('rclnodejs');
const TypeDescriptionService = require('rclnodejs/lib/type_description_service');
const DistroUtils = require('rclnodejs/lib/distro');
```

### Distribution Compatibility Check

```javascript
// Check if Type Description Service is supported
if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
  console.log(
    'Type Description Service is not supported in this ROS 2 version'
  );
  return;
}
```

### Node Creation and Spinning

```javascript
const node = rclnodejs.createNode('node_name');
rclnodejs.spin(node); // Use rclnodejs.spin(node), not node.spin()
```

### Type Hash Conversion

```javascript
// Get publisher info and convert type hash
const infos = node.getPublishersInfoByTopic('/topic_name', false);
const typeHash = TypeDescriptionService.toTypeHash(infos[0].topic_type_hash);
```

### Service Request Pattern

```javascript
const request = {
  type_name: 'std_msgs/msg/String',
  type_hash: typeHash,
  include_type_sources: true,
};

const client = node.createClient(
  'type_description_interfaces/srv/GetTypeDescription',
  '/node_name/get_type_description'
);

if (!(await client.waitForService(5000))) {
  throw new Error('Service not available');
}

// Request pattern used in tests (with timer for reliability)
const promise = new Promise((resolve, reject) => {
  const timer = setInterval(() => {
    client.sendRequest(request, (response) => {
      clearInterval(timer);
      if (response.successful) {
        // Process successful response
        resolve(response);
      } else {
        reject(new Error(`Request failed: ${response.failure_reason}`));
      }
    });
  }, 2000); // 2 second interval
});

await promise;
```

## Conclusion

The Type Description Service in rclnodejs provides powerful introspection capabilities for ROS 2 systems. By understanding how to query and use type information, you can:

- **Build dynamic tools** that adapt to different message types
- **Generate documentation** automatically from running systems
- **Debug communication issues** by understanding message structures
- **Create monitoring systems** that understand the data flowing through your robot

Remember to handle service availability gracefully and use appropriate error handling for robust applications.

For more information, see:

- [ROS 2 Introspection Documentation](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html)
- [rclnodejs API Documentation](https://robotwebtools.github.io/rclnodejs/)
- [Type Description Service implementation](../lib/type_description_service.js)
- [Type Description Service tests](../test/test-type-description-service.js)
- [Service examples](../example/services/)
