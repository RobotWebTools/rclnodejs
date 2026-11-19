# Error Handling Examples

The [example program](error-handling-example.mjs) exercises invalid arguments, parameter errors, timed-out and aborted requests, retry logic, and error logging.

## Run

With ROS sourced and `npm install` completed, run from the repository root:

```bash
node example/error-handling/error-handling-example.mjs
```

You need `example_interfaces`. See the [build requirements](../../docs/BUILDING.md#requirements) for checkout setup.

The program deliberately performs failing operations. It prints a heading for each section, catches the expected errors, and exits after `All examples completed`. It does not need a separate service server.

## Handle an Error

Use `instanceof` to distinguish errors such as `rclnodejs.TimeoutError` and `rclnodejs.AbortError`. Catch only errors you know how to handle; report or rethrow unexpected ones.

`RclNodeError` supplies a `code` and context when available. Subclasses add details, such as a timeout or an invalid argument. Not every failed operation produces a specialized rclnodejs error, so check the actual error rather than assuming a class from the operation's name.

See the [error classes](../../lib/errors.js) for the available fields and [the async service client](../services/client/async-client-example.mjs) for a request with a response timeout. A client timeout or abort stops the local wait; it does not cancel work already executing on the service server.