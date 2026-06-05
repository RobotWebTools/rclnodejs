# rostsd-gen

A node.js script that creates and updates the TypeScript interfaces.d.ts declaration file with type declarations for the generated interfaces (messages and services).

Run this script every time new interfaces are generated, see script/generate_messages.cjs

# run

You can update the interfaces.d.ts types manually by running the generate_tsd.cjs script.

```
node node_modules/rclnodejs/scripts/generate_tsd.cjs
```
