# rostsd-gen

Generates `types/interfaces.d.ts` for ROS 2 messages, services, and actions. It runs automatically with `npm install` and `npm run generate-messages`.

After adding interfaces, source their workspace and run from the repository root:

```bash
npm run generate-messages
```

Regenerate declarations only:

```bash
npm run generate-tsd-messages
```

For an installed package, run from your project directory:

```bash
node node_modules/rclnodejs/scripts/generate_tsd.cjs
```
