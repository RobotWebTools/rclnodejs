// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// From an installed package you would write `import rclnodejs from 'rclnodejs'`;
// run from this checkout we import the source entry point directly.

import rclnodejs from '../../index.js';

/**
 * Example 1: Type Validation Errors
 * Demonstrates catching TypeValidationError when providing wrong types
 */
async function example1_typeValidation() {
  console.log('\n=== Example 1: Type Validation ===');
  await rclnodejs.init();

  try {
    rclnodejs.createNode(123, 'namespace');
  } catch (error) {
    if (error instanceof rclnodejs.TypeValidationError) {
      console.log(
        `Expected ${error.expectedType}, got ${typeof error.providedValue}`
      );
    }
  }

  rclnodejs.shutdown();
}

/**
 * Example 2: Range Validation Errors
 * Demonstrates catching RangeValidationError for out-of-bounds values
 */
async function example2_rangeValidation() {
  console.log('\n=== Example 2: Range Validation ===');
  await rclnodejs.init();
  const node = rclnodejs.createNode('my_node');

  try {
    await node.createRate(2000);
  } catch (error) {
    if (error instanceof rclnodejs.RangeValidationError) {
      console.log(`Value ${error.providedValue} is ${error.validationRule}`);
    }
  }

  node.destroy();
  rclnodejs.shutdown();
}

/**
 * Example 3: Service Operation Errors
 * Demonstrates TimeoutError and AbortError handling
 */
async function example3_serviceErrors() {
  console.log('\n=== Example 3: Service Errors ===');
  await rclnodejs.init();
  const node = rclnodejs.createNode('client_node');
  const client = node.createClient(
    'example_interfaces/srv/AddTwoInts',
    'add_service'
  );

  try {
    await client.sendRequestAsync({ a: 1, b: 2 }, { timeout: 1 });
  } catch (error) {
    if (error instanceof rclnodejs.TimeoutError) {
      console.log(`${error.operationType} timed out after ${error.timeout}ms`);
    }
  }

  try {
    const controller = new AbortController();
    setTimeout(() => controller.abort(), 10);
    await client.sendRequestAsync(
      { a: 1, b: 2 },
      { signal: controller.signal }
    );
  } catch (error) {
    if (error instanceof rclnodejs.AbortError) {
      console.log(`${error.operationType} was aborted`);
    }
  }

  node.destroy();
  rclnodejs.shutdown();
}

/**
 * Example 4: Publisher Errors
 * Demonstrates catching PublisherError
 */
async function example4_publisherErrors() {
  console.log('\n=== Example 4: Publisher Errors ===');
  await rclnodejs.init();
  const node = rclnodejs.createNode('pub_node');

  try {
    node.createPublisher('InvalidMessageType', 'my_topic');
  } catch (error) {
    if (error instanceof rclnodejs.PublisherError) {
      console.log(`Publisher creation failed: ${error.message}`);
    }
  }

  node.destroy();
  rclnodejs.shutdown();
}

/**
 * Example 5: Subscription Errors
 * Demonstrates catching SubscriptionError
 */
async function example5_subscriptionErrors() {
  console.log('\n=== Example 5: Subscription Errors ===');
  await rclnodejs.init();
  const node = rclnodejs.createNode('sub_node');

  try {
    node.createSubscription('InvalidMessageType', 'my_topic', () => {});
  } catch (error) {
    if (error instanceof rclnodejs.SubscriptionError) {
      console.log(`Subscription creation failed: ${error.message}`);
    }
  }

  node.destroy();
  rclnodejs.shutdown();
}

/**
 * Example 6: Parameter Errors
 * Demonstrates ParameterTypeError and parameter-related errors
 */
async function example6_parameterErrors() {
  console.log('\n=== Example 6: Parameter Errors ===');
  await rclnodejs.init();
  const node = rclnodejs.createNode('param_node');

  try {
    new rclnodejs.Parameter(
      'test_param',
      rclnodejs.ParameterType.PARAMETER_STRING,
      123
    );
  } catch (error) {
    if (error instanceof rclnodejs.ParameterTypeError) {
      console.log(`Parameter type mismatch: ${error.message}`);
    }
  }

  node.destroy();
  rclnodejs.shutdown();
}

/**
 * Example 7: Name Validation Errors
 * Demonstrates catching NameValidationError for invalid ROS names
 */
async function example7_nameValidation() {
  console.log('\n=== Example 7: Name Validation ===');

  try {
    rclnodejs.validator.validateTopicName('invalid topic!');
  } catch (error) {
    if (error instanceof rclnodejs.NameValidationError) {
      console.log(
        `Invalid name at index ${error.invalidIndex}: ${error.message}`
      );
    }
  }
}

/**
 * Example 8: Error Recovery Pattern
 * Demonstrates retry logic with structured errors
 */
async function example8_errorRecovery() {
  console.log('\n=== Example 8: Error Recovery ===');
  await rclnodejs.init();
  const node = rclnodejs.createNode('recovery_node');
  const client = node.createClient(
    'example_interfaces/srv/AddTwoInts',
    'add_service'
  );

  const maxRetries = 3;
  let attempt = 0;

  while (attempt < maxRetries) {
    try {
      await client.sendRequestAsync({ a: 1, b: 2 }, { timeout: 100 });
      console.log('Success!');
      break;
    } catch (error) {
      attempt++;

      if (error instanceof rclnodejs.TimeoutError) {
        if (attempt < maxRetries) {
          console.log(`Retry ${attempt}/${maxRetries}`);
          continue;
        }
        console.log('All retries exhausted');
      } else if (error instanceof rclnodejs.AbortError) {
        console.log('User cancelled, no retry');
        break;
      } else {
        console.log('Unexpected error, no retry');
        break;
      }
    }
  }

  node.destroy();
  rclnodejs.shutdown();
}

/**
 * Example 9: Error Serialization
 * Demonstrates using toJSON() for logging/debugging
 */
async function example9_errorSerialization() {
  console.log('\n=== Example 9: Error Serialization ===');
  await rclnodejs.init();

  try {
    rclnodejs.createNode(null);
  } catch (error) {
    if (error instanceof rclnodejs.RclNodeError) {
      const errorData = error.toJSON();
      console.log('Error details:', JSON.stringify(errorData, null, 2));
    }
  }

  rclnodejs.shutdown();
}

/**
 * Example 10: Generic Error Handler
 * Demonstrates a reusable error handler for all rclnodejs errors
 */
function handleRclError(error, operation) {
  if (!(error instanceof rclnodejs.RclNodeError)) {
    console.error(`Unexpected error in ${operation}:`, error.message);
    return;
  }

  const context = [];
  if (error.nodeName) context.push(`node: ${error.nodeName}`);
  if (error.entityName)
    context.push(`${error.entityType}: ${error.entityName}`);

  console.error(
    `${error.name} in ${operation}${
      context.length ? ` (${context.join(', ')})` : ''
    }: ${error.message}`
  );

  if (error instanceof rclnodejs.TimeoutError) {
    console.error(`  Timeout: ${error.timeout}ms`);
  } else if (error instanceof rclnodejs.TypeValidationError) {
    console.error(`  Expected: ${error.expectedType}`);
  } else if (error instanceof rclnodejs.RangeValidationError) {
    console.error(`  Constraint: ${error.validationRule}`);
  }
}

async function example10_genericHandler() {
  console.log('\n=== Example 10: Generic Error Handler ===');
  await rclnodejs.init();

  try {
    rclnodejs.createNode(123);
  } catch (error) {
    handleRclError(error, 'createNode');
  }

  try {
    const node = rclnodejs.createNode('test_node');
    await node.createRate(5000);
  } catch (error) {
    handleRclError(error, 'createRate');
  }

  rclnodejs.shutdown();
}

try {
  await example1_typeValidation();
  await example2_rangeValidation();
  await example3_serviceErrors();
  await example4_publisherErrors();
  await example5_subscriptionErrors();
  await example6_parameterErrors();
  await example7_nameValidation();
  await example8_errorRecovery();
  await example9_errorSerialization();
  await example10_genericHandler();

  console.log('\n✅ All examples completed');
} catch (error) {
  console.error(error);
}
