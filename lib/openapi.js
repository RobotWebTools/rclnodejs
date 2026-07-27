// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
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

/**
 * OpenAPI 3.1 export for the Web Runtime capability registry.
 *
 * Turns `CapabilityRegistry.list()` into a documented, introspectable
 * Web API.
 *
 * Deliberately reuses the message introspection that already ships in
 * rclnodejs core (`lib/message_validation.js`'s `getMessageSchema()`)
 * instead of re-parsing rosidl ASTs. Resolving a
 * ROS type name is a pure local file lookup (`interface_loader` reads the
 * already-generated `.js` message classes) and never calls a live
 * `rclnodejs.init()`, so `buildOpenApiDocument()` can run standalone against
 * just a `web.json` config, without any transport or ROS graph. It still
 * needs ROS 2 sourced, though: `require()`-ing a generated message class
 * loads rclnodejs's native addon as a side effect, and without a sourced
 * environment the native loader may fail to match a prebuild and fall back
 * to a source rebuild.
 */

import interfaceLoader from './interface_loader.js';
import { getMessageSchema } from './message_validation.js';

/**
 * Map one ROS field-type descriptor (the shape produced by
 * `getMessageSchema()`, ultimately from the rosidl-generated
 * `ROSMessageDef`) to a JSON Schema fragment.
 *
 * 64-bit integers are the one deliberate deviation from the "obvious"
 * mapping: OpenAPI's `format: int64` is a documentation-only annotation on
 * top of `type: integer`, and JSON numbers cannot safely carry 64-bit
 * precision in JS (or most JSON parsers). rclnodejs's own wire convention
 * already represents `int64`/`uint64` as `"<digits>n"` strings (see
 * `PRIMITIVE_TYPE_MAP` in message_validation.js mapping them to `'bigint'`),
 * so the schema follows that convention rather than the nominal
 * `type: integer, format: int64` pairing.
 *
 * @param {object} fieldType - a field's `type` descriptor from
 *   `getMessageSchema(...).fields[i].type`
 * @param {Map<string,object>} components - accumulator for referenced
 *   nested-message component schemas, keyed by component name
 *   (`<pkg>__msg__<Type>` or `<pkg>__srv__<Type>_Request`/`_Response`)
 * @param {Set<string>} seen - cycle guard for recursive `$ref` resolution
 * @returns {object} a JSON Schema fragment
 */
function rosFieldTypeToJsonSchema(fieldType, components, seen) {
  if (fieldType.isArray) {
    const itemSchema = rosFieldTypeToJsonSchema(
      { ...fieldType, isArray: false },
      components,
      seen
    );
    const arraySchema = { type: 'array', items: itemSchema };
    if (fieldType.isFixedSizeArray && fieldType.arraySize != null) {
      arraySchema.minItems = fieldType.arraySize;
      arraySchema.maxItems = fieldType.arraySize;
    } else if (fieldType.isUpperBound && fieldType.arraySize != null) {
      arraySchema.maxItems = fieldType.arraySize;
    }
    return arraySchema;
  }

  if (fieldType.isPrimitiveType) {
    return primitiveToJsonSchema(fieldType);
  }

  // Nested message type — register as a component and return a $ref so
  // repeated uses of the same type (e.g. geometry_msgs/msg/Pose across many
  // capabilities) share one schema instead of being inlined N times.
  const componentName = `${fieldType.pkgName}__msg__${fieldType.type}`;
  const typeName = `${fieldType.pkgName}/msg/${fieldType.type}`;
  registerComponent(typeName, componentName, components, seen);
  return { $ref: `#/components/schemas/${componentName}` };
}

const INT64_TYPES = new Set(['int64', 'uint64']);

function primitiveToJsonSchema(fieldType) {
  const { type, stringUpperBound } = fieldType;

  if (type === 'bool') return { type: 'boolean' };
  if (INT64_TYPES.has(type)) {
    // See the docstring above — deliberately string, not integer. uint64
    // gets its own (unsigned-only) pattern/format so the schema doesn't
    // claim negative values are valid input for an unsigned field.
    const unsigned = type === 'uint64';
    return {
      type: 'string',
      format: unsigned ? 'uint64' : 'int64',
      pattern: unsigned ? '^[0-9]+n$' : '^-?[0-9]+n$',
      description: `ROS 2 ${type}, transmitted as a BigInt-string (e.g. "42n") for precision-safety.`,
    };
  }
  if (
    [
      'int8',
      'uint8',
      'int16',
      'uint16',
      'int32',
      'uint32',
      'byte',
      'char',
    ].includes(type)
  ) {
    return { type: 'integer' };
  }
  if (['float32', 'float64'].includes(type)) {
    return { type: 'number' };
  }
  if (type === 'string' || type === 'wstring') {
    const schema = { type: 'string' };
    if (stringUpperBound != null && stringUpperBound > 0) {
      schema.maxLength = stringUpperBound;
    }
    return schema;
  }
  // Unknown/unmapped primitive — fall back to permissive rather than
  // silently wrong.
  return {};
}

/**
 * Resolve a ROS message type name into a JSON Schema object (properties per
 * field), registering it into `components` under `componentName` so nested
 * `$ref`s can point at it. No-op if already registered/in-progress (cycle
 * guard).
 */
function registerComponent(typeName, componentName, components, seen) {
  if (components.has(componentName) || seen.has(componentName)) return;
  seen.add(componentName);

  let typeClass;
  try {
    typeClass = interfaceLoader.loadInterface(typeName);
  } catch {
    components.set(componentName, {
      type: 'object',
      description: `Could not resolve ${typeName}`,
    });
    return;
  }

  const schema = getMessageSchema(typeClass);
  components.set(
    componentName,
    messageSchemaToJsonSchema(schema, components, seen)
  );
}

/**
 * Convert a `getMessageSchema()`-shaped object into a JSON Schema Object
 * (`{type: 'object', properties: {...}}`), recursively registering any
 * nested message types into `components`.
 */
function messageSchemaToJsonSchema(schema, components, seen) {
  const properties = {};
  const required = [];
  for (const field of schema.fields || []) {
    if (field.name.startsWith('_')) continue;
    properties[field.name] = rosFieldTypeToJsonSchema(
      field.type,
      components,
      seen
    );
    required.push(field.name);
  }
  const jsonSchema = { type: 'object', properties };
  if (required.length) jsonSchema.required = required;
  if (schema.messageType) jsonSchema['x-ros-type'] = schema.messageType;
  return jsonSchema;
}

/**
 * Resolve a top-level capability type (message for publish/subscribe,
 * service Request/Response for call) to a JSON Schema, without registering
 * it as a component itself (the top-level request/response body is inlined
 * in the operation, only *nested* types become `$ref`d components — this
 * matches typical OpenAPI style for RPC-shaped APIs).
 */
function topLevelSchema(typeName, subType, components, seen) {
  let typeClass;
  try {
    typeClass = interfaceLoader.loadInterface(typeName);
  } catch {
    return { type: 'object', description: `Could not resolve ${typeName}` };
  }
  const resolved = subType ? typeClass[subType] : typeClass;
  const schema = getMessageSchema(resolved);
  if (!schema) {
    return { type: 'object', description: `Could not resolve ${typeName}` };
  }
  return messageSchemaToJsonSchema(schema, components, seen);
}

/**
 * Build a full OpenAPI 3.1 document from a capability registry snapshot
 * (`CapabilityRegistry.list()`'s shape: `{call, publish, subscribe}`, each a
 * `{name: typeName}` map).
 *
 * @param {{call: object, publish: object, subscribe: object}} capabilities
 * @param {object} [options]
 * @param {string} [options.title]
 * @param {string} [options.version]
 * @param {string} [options.basePath] - default '/capability'
 * @param {Array<{url: string, description?: string}>} [options.servers] -
 *   where the runtime's HTTP transport actually listens. Omitting this is
 *   rarely what you want: OpenAPI defaults `servers` to `[{url: '/'}]`, so
 *   a client resolves every path against whatever origin served the
 *   document. When the document is served by a *static* file server (the
 *   demo's `static.mjs` on :8080) but the runtime listens elsewhere
 *   (:9001), Swagger UI's "Try it out" would POST to the static server and
 *   get its 404 back. The CLI fills this in from the `http` config.
 * @returns {object} an OpenAPI 3.1 document (plain object; caller decides
 *   JSON vs. YAML serialization)
 */
function buildOpenApiDocument(capabilities, options = {}) {
  const {
    title = 'rclnodejs/web capability API',
    version = '0.0.0',
    basePath = '/capability',
    servers = [],
  } = options;

  const components = new Map();
  const seen = new Set();
  const paths = {};

  for (const [name, typeName] of Object.entries(capabilities.call || {})) {
    const route = `${basePath}/call${name}`;
    paths[route] = {
      post: {
        summary: `Call ROS 2 service ${name}`,
        operationId: `call_${sanitizeName(name)}`,
        'x-ros-capability': { kind: 'call', name, type: typeName },
        requestBody: {
          required: true,
          content: {
            'application/json': {
              schema: topLevelSchema(typeName, 'Request', components, seen),
            },
          },
        },
        responses: {
          200: {
            description: 'ROS 2 service response',
            content: {
              'application/json': {
                schema: topLevelSchema(typeName, 'Response', components, seen),
              },
            },
          },
          404: notExposedResponse(),
        },
      },
    };
  }

  for (const [name, typeName] of Object.entries(capabilities.publish || {})) {
    const route = `${basePath}/publish${name}`;
    paths[route] = {
      post: {
        summary: `Publish to ROS 2 topic ${name}`,
        operationId: `publish_${sanitizeName(name)}`,
        'x-ros-capability': { kind: 'publish', name, type: typeName },
        requestBody: {
          required: true,
          content: {
            'application/json': {
              schema: topLevelSchema(typeName, null, components, seen),
            },
          },
        },
        responses: {
          204: { description: 'Published, no content' },
          404: notExposedResponse(),
        },
      },
    };
  }

  for (const [name, typeName] of Object.entries(capabilities.subscribe || {})) {
    const route = `${basePath}/subscribe${name}`;
    paths[route] = {
      get: {
        summary: `Subscribe to ROS 2 topic ${name} via Server-Sent Events`,
        operationId: `subscribe_${sanitizeName(name)}`,
        'x-ros-capability': { kind: 'subscribe', name, type: typeName },
        description:
          'Requires the HTTP transport to be started with `sse: true` ' +
          '(`--http-sse` on the CLI). Shipped in rclnodejs 2.1.1. ' +
          '**Not testable via "Try it out"**: this response is an ' +
          'unbounded stream that never completes, and API explorers ' +
          '(e.g. Swagger UI) wait for the response body to finish before ' +
          'displaying it, so the request will appear to hang forever. Use ' +
          '`curl -N` or a browser `EventSource` instead.',
        responses: {
          200: {
            description: `Server-Sent Events stream of ${typeName} messages`,
            content: {
              'text/event-stream': {
                schema: topLevelSchema(typeName, null, components, seen),
              },
            },
          },
          404: notExposedResponse(),
        },
      },
    };
  }

  return {
    openapi: '3.1.0',
    info: { title, version },
    ...(servers.length ? { servers } : {}),
    paths,
    components: { schemas: Object.fromEntries(components) },
  };
}

function notExposedResponse() {
  return {
    description: 'Capability not exposed',
    content: {
      'application/json': {
        schema: {
          type: 'object',
          properties: {
            ok: { type: 'boolean', const: false },
            error: { type: 'string' },
            code: { type: 'string', const: 'not_exposed' },
          },
        },
      },
    },
  };
}

function sanitizeName(name) {
  return name.replace(/^\//, '').replace(/[^a-zA-Z0-9_]/g, '_');
}

export {
  buildOpenApiDocument,
  rosFieldTypeToJsonSchema,
  messageSchemaToJsonSchema,
  primitiveToJsonSchema,
};
