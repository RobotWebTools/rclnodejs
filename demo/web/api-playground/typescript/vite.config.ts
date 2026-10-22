import { defineConfig } from 'vite';

// Vite config for the typed rclnodejs Web API playground.
//
// `rclnodejs/web` is the public ESM entry exported from the
// `rclnodejs` npm package (see its package.json `exports` field), so
// Vite resolves it through `node_modules` with no alias required.
//
// Keep the ES2022 target consistent across development and production.
// The SDK loads Node's optional `ws` fallback lazily; browsers use
// `globalThis.WebSocket` without loading that fallback.
export default defineConfig({
  server: {
    // 8080 to match the JavaScript playground's static-file server, instead of
    // Vite's default 5173 — keeps the two demos' README instructions in sync.
    port: 8080,
  },
  esbuild: {
    target: 'es2022',
  },
  optimizeDeps: {
    esbuildOptions: {
      target: 'es2022',
    },
  },
  build: {
    target: 'es2022',
    sourcemap: true,
  },
});
