// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// TypeScript surface for the rclnodejs Web Runtime browser SDK.
//
// Goal: minimum setup. The user supplies one string per call — the
// ROS interface name — and the SDK derives the request, response, and
// message shapes from rclnodejs's auto-generated MessagesMap / ServicesMap.
//
//   const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
//     '/add_two_ints', { a: '2n', b: '40n' }   // typed
//   );
//
//   const sub = await ros.subscribe<'std_msgs/msg/String'>(
//     '/scan', (m) => render(m.data)            // m.data: string
//   );
//
// Untyped fallbacks (no generic) are kept so capabilities not in the
// generated maps still work.

declare module 'rclnodejs/web' {
  // -------- Wire-shape mapping ----------------------------------------

  /** Wire encoding of a ROS 2 64-bit integer field. */
  export type Int64Wire = `${number}n`;

  /**
   * Map an in-process rclnodejs message shape to its on-wire JSON shape.
   *
   * The Web Runtime serialises messages as JSON with one transformation:
   * 64-bit integer fields (`bigint` in the generated types) become the
   * string `"<n>n"` so they survive `JSON.stringify`. Everything else
   * passes through unchanged.
   *
   * Cases (checked in order):
   *   - `bigint`            → {@link Int64Wire} (the `"<n>n"` string)
   *   - `ReadonlyArray<U>`  → `WireType<U>[]`   (recurse per element)
   *   - `Date`              → `string`          (ISO string on the wire)
   *   - `object`            → field-wise recursion
   *   - everything else     → passes through unchanged
   *
   * @experimental — exposed for advanced consumers that want to derive
   * wire shapes by hand. Most users should rely on the type-name
   * generics on {@link RosClient.call} / `subscribe` / etc., which
   * apply this mapping internally.
   */
  // The bigint check uses `[T] extends [bigint]` (tuple wrapper) instead
  // of bare `T extends bigint` so the conditional doesn't distribute
  // across union members like `bigint | string` (which would map only
  // the bigint half and silently drop the rest).
  export type WireType<T> = [T] extends [bigint] ? Int64Wire : _WireRecurse<T>;

  /** Recursion step for {@link WireType}; pulled out to keep the cascade flat. */
  type _WireRecurse<T> =
    T extends ReadonlyArray<infer U>
      ? WireType<U>[]
      : T extends Date
        ? string
        : T extends object
          ? { [K in keyof T]: WireType<T[K]> }
          : T;

  // -------- Type-name lookup helpers ----------------------------------

  // The maps below are declared by rclnodejs's own .d.ts files via a
  // `declare module 'rclnodejs' { type MessagesMap = {...} }` augmentation.
  // Reaching them across the module boundary requires the inline
  // `import('rclnodejs')` form — a default-style `import type rclnodejs
  // from 'rclnodejs'` would resolve to the runtime's *value* export, not
  // the namespace, and `MessagesMap` would silently collapse to `never`.

  /** ROS 2 message type names available in the sourced environment. */
  export type MessageName = keyof import('rclnodejs').MessagesMap;

  /** ROS 2 service type names available in the sourced environment. */
  export type ServiceName = keyof import('rclnodejs').ServicesMap;

  /** Wire shape of the named message. `Msg<'std_msgs/msg/String'>` → `{ data: string }`. */
  export type Msg<TName extends MessageName> = WireType<
    import('rclnodejs').MessagesMap[TName]
  >;

  type _SvcCtor<TName extends ServiceName> =
    import('rclnodejs').ServicesMap[TName];

  /** Wire shape of the named service request. */
  export type SvcReq<TName extends ServiceName> = WireType<
    InstanceType<_SvcCtor<TName>['Request']>
  >;

  /** Wire shape of the named service response. */
  export type SvcResp<TName extends ServiceName> = WireType<
    InstanceType<_SvcCtor<TName>['Response']>
  >;

  // -------- SDK ------------------------------------------------------

  /**
   * Subscription handle returned by {@link RosClient.subscribe}.
   * Closing the handle cancels the subscription on the runtime side.
   */
  export interface Subscription {
    readonly subId: string;
    close(): Promise<void>;
  }

  export interface ConnectOptions {
    /**
     * Reopen the WebSocket link with backoff after a drop, replaying
     * subscriptions. Enables `'reconnecting'` / `'reconnected'` — see
     * {@link RosClient.on}; `'disconnected'` fires on any unexpected drop
     * regardless of this option. The first connect attempt still rejects
     * once rather than retrying forever.
     * @default false
     */
    reconnect?: boolean;
  }

  /** Detail passed to a `'reconnecting'` listener. */
  export interface ReconnectingDetail {
    /** 1-based attempt number for this reconnect sequence. */
    attempt: number;
    /** Delay in milliseconds before this attempt fires. */
    delay: number;
  }

  /** SDK lifecycle events emitted by {@link RosClient.on}, WebSocket link only. */
  export interface RosClientEventMap {
    /** An established connection dropped unexpectedly. */
    disconnected: undefined;
    /** About to retry opening the connection. */
    reconnecting: ReconnectingDetail;
    /** A retry succeeded and subscriptions were replayed. */
    reconnected: undefined;
  }

  /**
   * Pair of explicit endpoints when HTTP and WebSocket live behind
   * different URLs (e.g. one is fronted by a TLS proxy, the other isn't).
   * Pass either field on its own to limit the client to that transport.
   */
  export interface ConnectEndpoints {
    /** Base URL for the HTTP capability transport (`POST /capability/...`). */
    http?: string;
    /** Full URL of the WebSocket capability endpoint. */
    ws?: string;
  }

  /**
   * Browser-native Web Runtime client.
   *
   * The user-facing verb API (`call` / `publish` / `subscribe`) is the
   * same regardless of transport. The transport(s) used underneath
   * are picked from the URL scheme passed to {@link connect}:
   *
   *   - `ws://` / `wss://` — WebSocket only.
   *   - `http://` / `https://` — HTTP for `call`/`publish`; subscribe
   *     falls through to a sibling WebSocket endpoint at the same
   *     host with `/capability` appended.
   *   - {@link ConnectEndpoints} — both URLs spelled out.
   *
   * **Path conventions.** When a `ws://` / `wss://` URL is passed
   * without a path (or with just `/`), the SDK appends the runtime's
   * default `/capability` path automatically — so `'ws://host:9000'`
   * and `'ws://host:9000/capability'` behave identically. Pass an
   * explicit non-default path if your server changed `--path` /
   * `--http-base-path` or sits behind a path-rewriting proxy.
   */
  export class RosClient {
    readonly url: string;
    constructor(url: string | ConnectEndpoints, options?: ConnectOptions);
    connect(): Promise<this>;
    close(): Promise<void>;

    /** Subscribe to an SDK lifecycle event — see {@link RosClientEventMap}. */
    on<K extends keyof RosClientEventMap>(
      event: K,
      handler: (detail: RosClientEventMap[K]) => void
    ): this;

    /** Remove a listener added with {@link RosClient.on}. */
    off<K extends keyof RosClientEventMap>(
      event: K,
      handler: (detail: RosClientEventMap[K]) => void
    ): this;

    /**
     * Invoke a service capability.
     *
     * @example Typed via the ROS service type name (preferred)
     *   const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
     *     '/add_two_ints', { a: '2n', b: '40n' }
     *   );
     *   console.log(reply.sum);
     *
     * @example Untyped (capability with a custom type not in the
     *          generated maps, or quick prototyping)
     *   const reply = await ros.call('/whatever', { foo: 1 });
     */
    call<TName extends ServiceName>(
      capability: string,
      request: SvcReq<TName>
    ): Promise<SvcResp<TName>>;
    call(capability: string, request: unknown): Promise<unknown>;

    /** Publish a message to a topic capability. */
    publish<TName extends MessageName>(
      capability: string,
      message: Msg<TName>
    ): Promise<void>;
    publish(capability: string, message: unknown): Promise<void>;

    /** Subscribe to messages on a topic capability. */
    subscribe<TName extends MessageName>(
      capability: string,
      callback: (msg: Msg<TName>) => void
    ): Promise<Subscription>;
    subscribe(
      capability: string,
      callback: (msg: unknown) => void
    ): Promise<Subscription>;
  }

  /**
   * Open a connection to a Web Runtime capability endpoint.
   * Convenience wrapper around `new RosClient(...).connect()`.
   *
   * @param url Either a single URL (`ws://`, `wss://`, `http://`,
   *            `https://`) or a {@link ConnectEndpoints} pair.
   */
  export function connect(
    url: string | ConnectEndpoints,
    options?: ConnectOptions
  ): Promise<RosClient>;
}
