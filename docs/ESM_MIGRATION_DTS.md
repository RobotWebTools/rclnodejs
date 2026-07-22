# ESM Migration: TypeScript Declaration (`.d.ts`) Modernization

> Status: planned (a follow-up step of the ESM migration, after the dual
> ESM + CommonJS build). This document captures the reasoning behind
> converting the `types/*.d.ts` files from the legacy ambient
> `declare module 'rclnodejs'` style to real ESM declaration modules.

## TL;DR

- The runtime is already native ESM with a dual ESM + CommonJS build, and
  `package.json` exposes typed entry points through its `exports` map.
- The ~40 files in `types/*.d.ts` still use the **legacy ambient pattern**:
  every file is wrapped in `declare module 'rclnodejs' { ... }` and the
  barrel `types/index.d.ts` stitches them together with
  `/// <reference path="..." />` directives.
- This **still works today** for the bare `import ... from 'rclnodejs'`
  because an ambient `declare module 'name'` is matched globally by the
  literal string, independent of `moduleResolution`.
- It is **not** broken, but it is legacy and incomplete: it does not
  cleanly serve modern consumers on `node16`/`nodenext`/`bundler`
  resolution, especially for **subpath entries** (`rclnodejs/web`,
  `rclnodejs/web/server`, `rclnodejs/rosocket`) and the **default-import /
  CJS-vs-ESM interop shape**.
- The modernization converts each declaration file into a real ESM module
  (`export` / `export default`, `export * from './x.js'`) whose module
  identity comes from `package.json` (`name` + `exports.types`), exactly
  like the runtime already resolves.

## Background: two ways to bind types to the name `rclnodejs`

### A. Today — the name is hard-coded inside the `.d.ts`

```typescript
// types/clock_type.d.ts (current)
declare module 'rclnodejs' {
  enum ClockType {
    ROS_TIME = 1,
    SYSTEM_TIME = 2,
    STEADY_TIME = 3,
  }
}
```

`declare module 'rclnodejs'` is a **declaration-merging, ambient** construct.
Every file re-opens the same named block; TypeScript merges them all into a
single global surface. The barrel file glues them with triple-slash refs:

```typescript
// types/index.d.ts (current)
/// <reference path="./base.d.ts" />
/// <reference path="./clock_event.d.ts" />
// ...more /// <reference> lines...

import { ChildProcess } from 'child_process';

declare module 'rclnodejs' {
  function createNode(/* ... */): Node;
  // ...everything merged into the same global block...
}
```

This is the same idiom `@types/*` packages use to describe a **third-party**
library that did not ship its own types. `rclnodejs` ships its own types, so
it is technically using the "describe someone else's library" pattern to
describe itself.

### B. After — the name comes from `package.json`

A package that ships its own types does not name itself in the `.d.ts`. The
identity `rclnodejs` is established by `package.json`, and the type entry is
found through the `exports` `types` conditions:

```jsonc
{
  "name": "rclnodejs", // defines the module name
  "exports": {
    ".": {
      "types": "./types/index.d.ts",
      "import": "./dist/index.js",
      "require": "./dist/index.cjs",
    },
    "./web": {
      "types": "./web/index.d.ts",
      "import": "./dist/web.js",
      "require": "./dist/web.cjs",
    },
    "./web/server": {
      "types": "./lib/runtime/index.d.ts",
      "import": "./dist/server.js",
      "require": "./dist/server.cjs",
    },
    "./rosocket": {
      "import": "./dist/rosocket.js",
      "require": "./dist/rosocket.cjs",
    },
  },
}
```

Resolution for `import { ClockType } from 'rclnodejs'`:

```
import ... from 'rclnodejs'
  -> read node_modules/rclnodejs/package.json
  -> "name" === "rclnodejs"            (matches the bare specifier)
  -> exports["."]["types"]            (-> ./types/index.d.ts)
  -> that file's `export`s are the public type surface
```

The declaration files just `export` their symbols. Nothing inside them says
the string `rclnodejs` — the binding is done by the folder + `package.json`,
exactly like the runtime JS already works.

## Does TypeScript currently work? (the nuance)

**Yes, for the bare import.** An ambient `declare module 'rclnodejs'` is
matched by literal name regardless of `moduleResolution` (`node`, `node16`,
`nodenext`, or `bundler`). The project's own `tsconfig.json` already uses
`"moduleResolution": "nodenext"`, and the `tsd` type tests pass under it.

What is **fragile or missing** for a consumer on `node16`/`nodenext`/`bundler`:

| Consumer usage                                       | Outcome with current ambient types                                                                        |
| ---------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `import * as rclnodejs from 'rclnodejs'` (namespace) | works — global ambient match by name                                                                      |
| `import rclnodejs from 'rclnodejs'` (default)        | may be wrong — CJS vs ESM interop shape is not cleanly described                                          |
| `import { X } from 'rclnodejs/web'` (subpath)        | likely untyped — a single `declare module 'rclnodejs'` says nothing about the `'rclnodejs/web'` specifier |

Classic `"moduleResolution": "node"` does not read `exports` at all (only the
top-level `"types"`), so the global ambient trick was always "good enough"
there. Modern resolution is strict and path/file-based and honors the
`exports` conditions — which is what the dual build and subpaths require.

### Two different `tsconfig.json` files

- `rclnodejs/tsconfig.json` (`nodenext`) governs how **rclnodejs checks
  itself** (build + `tsd`). It does **not** ship to consumers.
- The **consumer's** `tsconfig.json` governs how the **shipped** `.d.ts`
  files are resolved. That is the config Phase-4-style declarations must
  satisfy, and it is the one the maintainers do not control.

## Why CommonJS did not "force" naming via `package.json`

It could have. The name was always definable via the top-level `"types"`
field. The ambient `declare module` idiom was a **choice** because:

1. It made splitting one type surface across ~40 files trivial via
   declaration merging (no explicit `export * from './x'` wiring needed).
2. Classic `node` resolution ignores `exports`, so a global name-matched
   block "just worked" everywhere with no resolution concerns.
3. CommonJS's file-based export shape would need `export =` (the CJS
   counterpart of ESM's `export default`), which the ambient block avoided
   having to express.

ESM removes the cover that made the ambient trick sufficient: `nodenext`
resolution is path-based, honors `exports`, and the dual build exposes
subpaths — so the types must become real modules.

## The change, by example

### Leaf file — unwrap and export

```typescript
// types/clock_type.d.ts (after)
export enum ClockType {
  ROS_TIME = 1,
  SYSTEM_TIME = 2,
  STEADY_TIME = 3,
}
```

### File with a namespace — export the namespace

```typescript
// types/distro.d.ts (after)
export namespace DistroUtils {
  type DistroName = 'eloquent' | 'foxy' | /* ... */ 'rolling';
  enum DistroId {
    /* ... */
  }
  function getDistroId(distroName?: DistroName): DistroId;
  function getDistroName(distroId?: DistroId): string | undefined;
}
```

### Barrel `index.d.ts` — re-export modules and expose a default

```typescript
// types/index.d.ts (after)
import { ChildProcess } from 'child_process';

export * from './clock_type.js';
export * from './distro.js';
export * from './node.js';
export * from './parameter.js';
// ...one re-export per file...

declare function createNode(/* ... */): Node;
// ...other top-level functions...

// Describes both `import rclnodejs from 'rclnodejs'` and the CJS
// `const rclnodejs = require('rclnodejs')` shape produced by the dual build.
declare const rclnodejs: {
  createNode: typeof createNode; /* ...namespace... */
};
export default rclnodejs;
```

### Mechanical summary

| Aspect       | Before                               | After                                           |
| ------------ | ------------------------------------ | ----------------------------------------------- |
| Wrapper      | `declare module 'rclnodejs' { ... }` | removed                                         |
| Symbols      | implicitly merged, not exported      | explicit `export`                               |
| File linking | `/// <reference path="..." />`       | `export * from './x.js'`                        |
| Public shape | one global ambient name              | per-file module + `export default` for the root |
| Resolved by  | literal string `'rclnodejs'`         | the `exports` `types` condition (by path)       |

## Why this is a separate step

The change is largely **mechanical** (unwrap each file, add `export`), but it
must be applied **consistently** across all ~40 files, and the barrel
`index.d.ts` must switch from `/// <reference>` stitching to real
`export * ` re-exports plus an `export default`. Getting the default-import
shape and every subpath entry to type-check correctly under `nodenext` is
semantically delicate, so it is best reviewed independently from the
build-system change.
