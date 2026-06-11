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

const { ParameterType, Parameter, ParameterDescriptor } = rclnodejs;

await rclnodejs.init();

const targetNode = rclnodejs.createNode('target_node');
const clientNode = rclnodejs.createNode('client_node');

targetNode.declareParameter(
  new Parameter('max_speed', ParameterType.PARAMETER_DOUBLE, 10.5),
  new ParameterDescriptor(
    'max_speed',
    ParameterType.PARAMETER_DOUBLE,
    'Maximum speed in m/s'
  )
);

targetNode.declareParameter(
  new Parameter('debug_mode', ParameterType.PARAMETER_BOOL, false),
  new ParameterDescriptor(
    'debug_mode',
    ParameterType.PARAMETER_BOOL,
    'Enable debug logging'
  )
);

targetNode.declareParameter(
  new Parameter('retry_count', ParameterType.PARAMETER_INTEGER, BigInt(3)),
  new ParameterDescriptor(
    'retry_count',
    ParameterType.PARAMETER_INTEGER,
    'Number of retries'
  )
);

rclnodejs.spin(targetNode);
rclnodejs.spin(clientNode);

const paramClient = clientNode.createParameterClient('target_node');

try {
  await paramClient.waitForService(10000);

  const { names } = await paramClient.listParameters();
  console.log('Available parameters:', names);

  const maxSpeed = await paramClient.getParameter('max_speed');
  console.log(`max_speed = ${maxSpeed.value}`);

  const params = await paramClient.getParameters([
    'max_speed',
    'debug_mode',
    'retry_count',
  ]);
  console.log(
    'Retrieved parameters:',
    params.map((p) => p.name)
  );

  await paramClient.setParameter('max_speed', 15.0);
  await paramClient.setParameters([
    { name: 'debug_mode', value: true },
    { name: 'retry_count', value: 5 },
  ]);

  const descriptors = await paramClient.describeParameters(['max_speed']);
  console.log(`max_speed descriptor:`, descriptors[0]);

  try {
    await paramClient.getParameter('max_speed', { timeout: 1 });
  } catch (error) {
    // Expected timeout with 1ms
  }
} catch (error) {
  console.error('Error:', error.message);
} finally {
  clientNode.destroy();
  targetNode.destroy();
  rclnodejs.shutdown();
}
