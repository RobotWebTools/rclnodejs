# Vendored ref helpers

These ref helpers are maintained with rclnodejs. The existing `ref-napi` implementation contains local integration changes; the array and struct adapters' production JavaScript is copied without functional or formatting changes from the published forks below.

## Why these adapters live here

The two array and struct adapters are maintained by the rclnodejs team and used only by rclnodejs. Keeping them in separate npm packages adds repository, versioning, testing, and publication work without serving an independent consumer.

Maintaining them here lets changes to the adapters, the local `ref-napi` implementation, and generated messages be reviewed and tested together, then shipped in one rclnodejs release. This repository is the development location for these copies, avoiding parallel active implementations in the standalone repositories.

This is maintenance consolidation, not a claim that the packages are abandoned or that vendoring improves runtime performance. The adapters are pure JavaScript: moving them does not remove a native build or ABI boundary, and the `array-index` dependency chain remains.

## Sources and integration

| Directory                        | Upstream source                                             | Provenance                                                                                                                               |
| -------------------------------- | ----------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| [ref-napi](./ref-napi)           | [rclnodejs/ref-napi](https://github.com/rclnodejs/ref-napi) | Adapted in-tree implementation, not an unmodified upstream snapshot.                                                                     |
| [ref-array-di](./ref-array-di)   | `@rclnodejs/ref-array-di@1.2.2`                             | [`e967bbab6f0bc1f6e1213d37cf9bccaa12006907`](https://github.com/rclnodejs/ref-array-di/commit/e967bbab6f0bc1f6e1213d37cf9bccaa12006907)  |
| [ref-struct-di](./ref-struct-di) | `@rclnodejs/ref-struct-di@1.1.1`                            | [`e5cb861478133ef355e5702bf546189377825ad2`](https://github.com/rclnodejs/ref-struct-di/commit/e5cb861478133ef355e5702bf546189377825ad2) |

The `ref-napi` JavaScript layer uses native bindings compiled into `rclnodejs.node` through [binding.gyp](../binding.gyp), rather than a separately installed native addon. Its local package manifest defines a CommonJS boundary.

The `ref-array-di` and `ref-struct-di` directories retain their original MIT licenses and copyright notices. Their private package manifests define CommonJS boundaries inside rclnodejs's ESM package; these directories are not independently published.

Both factories still receive the same local `ref-napi` implementation, including its `PointerBuffer` support. Their remaining dependencies, `array-index` and `debug`, are declared in the root [package.json](../package.json). Vendoring does not remove the transitive dependencies of `array-index`.

When updating these adapters:

- Preserve their licenses and update the source version/commit above.
- Keep layout, alignment, buffer ownership, and dependency-injection behavior unchanged unless an intentional change is separately reviewed.
- Run the ref-type, array, serialization, raw-message, and installed-package tests.
- When changing imports in the [message template](../rosidl_gen/templates/message-template.cjs), regenerate existing interfaces. Normal installations do this through `postinstall`; source checkouts that skip installation hooks must run `npm run generate-messages` with ROS sourced. Bump the [generator version](../rosidl_gen/generator.json) if automatic cache invalidation is required.

Existing published versions of the scoped npm packages must remain available because older rclnodejs releases still depend on them. Moving current development here does not unpublish or deprecate those versions.
