# TypeScript Services Demo

The [client](src/client.ts) waits for the [server](src/server.ts), calls `/add_two_ints` every three seconds, and prints each sum as the reply arrives.

## Run

After [TypeScript setup](../README.md#setup), run from the repository root:

```bash
cd demo/typescript/services
npm install --ignore-scripts
npm run start:both
```

`start:both` builds and runs both nodes. To run them separately, start `npm run start:server`, then `npm run start:client` in another terminal in the demo directory. Stop them with Ctrl+C.

## Call the Service

Leave the server running and call it from another terminal:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 42, b: 17}'
```

The response contains `sum=59`. The `a`, `b`, and `sum` fields are signed 64-bit integers. Use `bigint` values such as `42n` in TypeScript. `createClient()` infers the types from the constructor returned by `rclnodejs.require()`.

Set `REQUEST_INTERVAL` in the client to `1_000_000_000n` for one request per second.