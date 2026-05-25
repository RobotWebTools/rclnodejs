import { defineConfig } from 'vite';

// Vite config for the rclnodejs Web Runtime TypeScript demo.
//
// `rclnodejs/web` is the public ESM entry exported from the
// `rclnodejs` npm package (see its package.json `exports` field), so
// Vite resolves it through `node_modules` with no alias required.
//
// The browser SDK uses a top-level `await import('ws')` to optionally
// pull in a Node WebSocket polyfill (no-op in real browsers). That
// requires an ES2022 build target both for the production bundle
// (`build.target`) and for esbuild's dev-mode dependency pre-bundling
// (`optimizeDeps.esbuildOptions.target`).
export default defineConfig({
  server: {
    // 8080 to match the JavaScript demo's static-file server, instead of
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
