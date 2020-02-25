# rostsd-gen

A node.js script that creates and updates the TypeScript interfaces.d.ts declaration file with type declarations for the generated interfaces (messages and services).

Run this script everytime new interfaces are generated, see script/generate_messages.js

# run

You can update the interfaces.d.ts types manually by running the generate_tsd.js script.

```
node node_modules/rclnodejs/scripts/generate_tsd.js
```
