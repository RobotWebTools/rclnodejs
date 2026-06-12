// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Type surface for the `rclnodejs/web/server` subpath export.
//
// The server runtime composes a CapabilityRegistry, a Dispatcher, and one or
// more transport adapters around an existing rclnodejs Node. The shapes
// declared here mirror the JavaScript classes in this directory; only the
// public, supported surface is typed.

import type { Node } from 'rclnodejs';

export type CapabilityKind = 'call' | 'publish' | 'subscribe';

/** A single capability entry as resolved from the registry. */
export interface Capability {
  kind: CapabilityKind;
  name: string;
  type: string;
}

/** Shorthand or rich form accepted by `expose()`. */
export type CapabilitySpecValue = string | { type: string };

/** Per-kind map from ROS name → type or rich descriptor. */
export type CapabilityMap = Record<string, CapabilitySpecValue>;

export interface ExposeSpec {
  call?: CapabilityMap;
  publish?: CapabilityMap;
  subscribe?: CapabilityMap;
}

/** Snapshot returned by `CapabilityRegistry.list()`. */
export interface RegistrySnapshot {
  call: Record<string, string>;
  publish: Record<string, string>;
  subscribe: Record<string, string>;
}

/**
 * Declarative allow-list of ROS 2 capabilities exposed to the Web Runtime.
 * Capabilities not registered here are rejected before any Node API call.
 */
export class CapabilityRegistry {
  expose(spec?: ExposeSpec): this;
  resolve(kind: CapabilityKind, name: string): Capability | null;
  list(): RegistrySnapshot;
}

/** Per-connection handler installed onto a Connection by the runtime. */
export class Dispatcher {
  constructor(options: { node: Node; registry: CapabilityRegistry });
  handle(connection: Connection): void;
}

/** Wire frame exchanged between a transport and the dispatcher. */
export interface CapabilityFrame {
  id?: string | number;
  kind?: 'call' | 'publish' | 'subscribe' | 'unsubscribe' | 'action';
  capability?: string;
  payload?: unknown;
  subId?: string | number;
  ok?: boolean;
  event?: string;
  error?: string;
  code?: string;
}

/** Per-connection abstraction provided by transport adapters. */
export class Connection {
  send(frame: CapabilityFrame): void;
  close(code?: number, reason?: string): void;
  on(event: 'message', listener: (frame: CapabilityFrame) => void): this;
  on(event: 'close', listener: () => void): this;
  emit(event: 'message', frame: CapabilityFrame): boolean;
  emit(event: 'close'): boolean;
}

/** Layer-2 transport adapter base class. */
export class TransportAdapter {
  start(options: {
    onConnection: (connection: Connection) => void;
  }): Promise<{ address: string; port: number }>;
  stop(): Promise<void>;
}

export interface WebSocketTransportOptions {
  port?: number;
  host?: string;
  path?: string;
  verifyClient?: (req: import('http').IncomingMessage) => boolean;
}

export class WebSocketTransport extends TransportAdapter {
  constructor(options?: WebSocketTransportOptions);
  port: number;
  host: string;
  path: string;
}

export interface HttpTransportOptions {
  port?: number;
  host?: string;
  basePath?: string;
  sse?: boolean;
  sseKeepAliveMs?: number;
  cors?: boolean | string | string[];
  verifyRequest?: (req: import('http').IncomingMessage) => boolean;
}

export class HttpTransport extends TransportAdapter {
  constructor(options?: HttpTransportOptions);
  port: number;
  host: string;
  basePath: string;
}

export interface CreateRuntimeOptions {
  node: Node;
  transport?: TransportAdapter;
  transports?: TransportAdapter[];
}

export class Runtime {
  readonly node: Node;
  readonly registry: CapabilityRegistry;
  readonly dispatcher: Dispatcher;
  readonly transports: TransportAdapter[];
  expose(spec: ExposeSpec): this;
  start(): Promise<this>;
  stop(): Promise<void>;
}

/**
 * Convenience factory. Defaults to a single {@link WebSocketTransport} on
 * port 9000, path `/capability`.
 */
export function createRuntime(options: CreateRuntimeOptions): Runtime;
