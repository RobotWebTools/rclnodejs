import assert from 'assert';
import sinon from 'sinon';

import fs from 'fs';
import child_process from 'child_process';
import Module from 'node:module';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const packageRoot = fileURLToPath(new URL('../', import.meta.url));
const localDirectories = [
  ['build', 'Debug'],
  ['build', 'Release'],
];

// native_loader attaches TestHelpers to the addon only when NODE_ENV==='test'.
// In ESM modules are singletons and cannot be re-required via require.cache, so
// set the env first and use a cache-busting dynamic import to obtain a fresh
// module instance that exposes TestHelpers.
process.env.NODE_ENV = 'test';
const nativeLoader = (await import('../lib/native_loader.js?env=test')).default;

describe('NativeLoader testing', function () {
  const sandbox = sinon.createSandbox();
  let originalPlatform;
  let originalArch;
  let originalEnv;
  let loader;

  beforeEach(function () {
    originalPlatform = process.platform;
    originalArch = process.arch;
    originalEnv = { ...process.env };
    loader = nativeLoader.TestHelpers;
    delete process.env.NODE_BINDINGS_ARROW;
    delete process.env.NODE_BINDINGS_COMPILED_DIR;
    delete process.env.RCLNODEJS_FORCE_BUILD;
  });

  afterEach(function () {
    sandbox.restore();
    Object.defineProperty(process, 'platform', { value: originalPlatform });
    Object.defineProperty(process, 'arch', { value: originalArch });
    process.env = originalEnv;
  });

  function stubLocalAddons(available, load = () => ({})) {
    const resolveFilename = Module._resolveFilename;
    const loadModule = Module._load;
    const isAddon = (request) =>
      typeof request === 'string' &&
      path.isAbsolute(request) &&
      path.basename(request) === 'rclnodejs.node';
    const resolve = sandbox
      .stub(Module, '_resolveFilename')
      .callsFake(function (request, ...args) {
        if (!isAddon(request))
          return resolveFilename.call(this, request, ...args);
        if (available(request)) return request;
        throw Object.assign(new Error(`Missing native addon: ${request}`), {
          code: 'MODULE_NOT_FOUND',
        });
      });
    const requireAddon = sandbox.spy(load);
    sandbox.stub(Module, '_load').callsFake(function (request, ...args) {
      if (isAddon(request)) {
        Module._resolveFilename(request, ...args);
        return requireAddon(request);
      }
      return loadModule.call(this, request, ...args);
    });
    return { resolve, requireAddon };
  }

  it('reports only node-gyp Debug and Release paths in that order', function () {
    const expected = localDirectories.map((directory) =>
      path.join(packageRoot, ...directory, 'rclnodejs.node')
    );
    const { requireAddon } = stubLocalAddons(() => false);
    assert.throws(
      () => loader.loadLocalAddon(),
      (error) => {
        assert.strictEqual(error.code, undefined);
        assert.deepStrictEqual(error.tries, expected);
        assert.strictEqual(
          error.message,
          'Could not locate the bindings file. Tried:\n' +
            expected.map((candidate) => ' \u2192 ' + candidate).join('\n')
        );
        return true;
      }
    );
    assert.strictEqual(requireAddon.callCount, 0);
  });

  it('honors the diagnostic arrow override', function () {
    process.env.NODE_BINDINGS_ARROW = ' -> ';
    stubLocalAddons(() => false);
    assert.throws(
      () => loader.loadLocalAddon(),
      (error) => {
        assert.strictEqual(
          error.message,
          'Could not locate the bindings file. Tried:\n' +
            error.tries.map((candidate) => ' -> ' + candidate).join('\n')
        );
        return true;
      }
    );
  });

  for (const directory of localDirectories) {
    it(`loads a local addon from ${directory.join('/')}`, function () {
      const expected = path.join(packageRoot, ...directory, 'rclnodejs.node');
      const addon = {};
      const { requireAddon } = stubLocalAddons(
        (candidate) => candidate === expected,
        () => addon
      );
      assert.strictEqual(loader.loadLocalAddon(), addon);
      assert.strictEqual(addon.path, expected);
      assert.strictEqual(requireAddon.callCount, 1);
      assert.strictEqual(requireAddon.firstCall.args[0], expected);
    });
  }

  it('prefers Debug over Release when both local builds exist', function () {
    const debugPath = path.join(
      packageRoot,
      'build',
      'Debug',
      'rclnodejs.node'
    );
    const releasePath = path.join(
      packageRoot,
      'build',
      'Release',
      'rclnodejs.node'
    );
    const { requireAddon } = stubLocalAddons((candidate) =>
      [debugPath, releasePath].includes(candidate)
    );
    assert.strictEqual(loader.loadLocalAddon().path, debugPath);
    assert.strictEqual(requireAddon.callCount, 1);
    assert.strictEqual(requireAddon.firstCall.args[0], debugPath);
  });

  it('does not search custom compiled directories', function () {
    process.env.NODE_BINDINGS_COMPILED_DIR = 'custom-compiled';
    const legacyPath = path.join(
      packageRoot,
      'custom-compiled',
      process.versions.node,
      process.platform,
      process.arch,
      'rclnodejs.node'
    );
    const expected = localDirectories.map((directory) =>
      path.join(packageRoot, ...directory, 'rclnodejs.node')
    );
    const { resolve, requireAddon } = stubLocalAddons(
      (candidate) => candidate === legacyPath
    );
    assert.throws(
      () => loader.loadLocalAddon(),
      (error) => {
        assert.deepStrictEqual(error.tries, expected);
        return true;
      }
    );
    assert.deepStrictEqual(
      resolve.getCalls().map((call) => call.args[0]),
      expected
    );
    assert.strictEqual(requireAddon.callCount, 0);
  });

  for (const code of ['ERR_DLOPEN_FAILED', 'ERR_DLOPEN_DISABLED']) {
    it(`preserves ${code} from an existing addon instead of probing later paths`, function () {
      const failure = Object.assign(
        new Error('Native dependency failed to load'),
        { code }
      );
      const { resolve, requireAddon } = stubLocalAddons(
        () => true,
        () => {
          throw failure;
        }
      );
      assert.throws(
        () => loader.loadLocalAddon(),
        (error) => error === failure
      );
      assert.strictEqual(resolve.callCount, 1);
      assert.strictEqual(requireAddon.callCount, 1);
    });
  }

  for (const [label, code, message] of [
    ['missing module', 'MODULE_NOT_FOUND', 'Native dependency is missing'],
    ['qualified path', 'QUALIFIED_PATH_RESOLUTION_FAILED', 'Resolution failed'],
    [
      'missing library',
      'ERR_DLOPEN_FAILED',
      'Could NOT FIND dependent library',
    ],
    [
      'missing library without code',
      undefined,
      'Could not find native library',
    ],
  ]) {
    it(`loads Release after a recoverable Debug error (${label}) without rebuilding`, async function () {
      Object.defineProperty(process, 'platform', { value: 'darwin' });
      const debugPath = path.join(
        packageRoot,
        'build',
        'Debug',
        'rclnodejs.node'
      );
      const releasePath = path.join(
        packageRoot,
        'build',
        'Release',
        'rclnodejs.node'
      );
      const failure = Object.assign(new Error(message), { code });
      const addon = {};
      const { requireAddon } = stubLocalAddons(
        (candidate) => [debugPath, releasePath].includes(candidate),
        (candidate) => {
          if (candidate === debugPath) throw failure;
          return addon;
        }
      );
      const execSync = sandbox
        .stub(child_process, 'execSync')
        .throws(
          new Error('A working Release addon must not require rebuilding')
        );
      const loaded = await import(
        `../lib/native_loader.js?recoverable=${encodeURIComponent(label)}`
      );
      assert.strictEqual(loaded.default, addon);
      assert.strictEqual(addon.path, releasePath);
      assert.deepStrictEqual(
        requireAddon.getCalls().map((call) => call.args[0]),
        [debugPath, releasePath]
      );
      assert.strictEqual(execSync.callCount, 0);
    });
  }

  it('reports the full search after all existing addons have recoverable errors', function () {
    const expected = localDirectories.map((directory) =>
      path.join(packageRoot, ...directory, 'rclnodejs.node')
    );
    const { requireAddon } = stubLocalAddons(
      () => true,
      () => {
        throw Object.assign(new Error('Native dependency is missing'), {
          code: 'MODULE_NOT_FOUND',
        });
      }
    );
    assert.throws(
      () => loader.loadLocalAddon(),
      (error) => {
        assert.deepStrictEqual(error.tries, expected);
        return true;
      }
    );
    assert.strictEqual(requireAddon.callCount, expected.length);
  });

  it('preserves unexpected resolution errors', function () {
    const failure = Object.assign(new Error('Permission denied'), {
      code: 'EACCES',
    });
    const { requireAddon } = stubLocalAddons(() => {
      throw failure;
    });
    assert.throws(
      () => loader.loadLocalAddon(),
      (error) => error === failure
    );
    assert.strictEqual(requireAddon.callCount, 0);
  });

  for (const state of ['missing', 'unloadable']) {
    it(`rebuilds once when the local addon is ${state} and then loads the result`, async function () {
      Object.defineProperty(process, 'platform', { value: 'darwin' });
      let built = false;
      const expected = path.join(
        packageRoot,
        'build',
        'Release',
        'rclnodejs.node'
      );
      const addon = {};
      stubLocalAddons(
        (candidate) =>
          candidate === expected && (built || state === 'unloadable'),
        () => {
          if (!built) {
            throw Object.assign(new Error('Native dependency failed to load'), {
              code: 'ERR_DLOPEN_FAILED',
            });
          }
          return addon;
        }
      );
      const execSync = sandbox.stub(child_process, 'execSync').callsFake(() => {
        built = true;
      });
      const loaded = await import(`../lib/native_loader.js?local=${state}`);
      assert.strictEqual(loaded.default, addon);
      assert.strictEqual(addon.path, expected);
      assert.ok(execSync.calledOnce);
      assert.strictEqual(execSync.firstCall.args[0], 'npm run rebuild');
      assert.strictEqual(
        execSync.firstCall.args[1].cwd,
        path.resolve(packageRoot)
      );
    });
  }

  it('surfaces a rebuild failure as a NativeError', async function () {
    Object.defineProperty(process, 'platform', { value: 'darwin' });
    stubLocalAddons(() => false);
    const failure = new Error('test rebuild failed');
    sandbox.stub(child_process, 'execSync').throws(failure);
    await assert.rejects(
      import('../lib/native_loader.js?rebuild-failed=1'),
      /Failed to build rclnodejs from source: test rebuild failed/
    );
  });

  it('customFallbackLoader returns null on non-linux', function () {
    Object.defineProperty(process, 'platform', { value: 'win32' });
    assert.strictEqual(loader.customFallbackLoader(), null);
  });

  it('customFallbackLoader returns null if env info missing', function () {
    Object.defineProperty(process, 'platform', { value: 'linux' });
    process.env.ROS_DISTRO = '';
    assert.strictEqual(loader.customFallbackLoader(), null);
  });

  it('customFallbackLoader returns null if prebuild dir not found', function () {
    Object.defineProperty(process, 'platform', { value: 'linux' });
    Object.defineProperty(process, 'arch', { value: 'x64' });
    process.env.ROS_DISTRO = 'humble';

    sandbox.stub(fs, 'existsSync').returns(false);
    assert.strictEqual(loader.customFallbackLoader(), null);
  });

  it('customFallbackLoader attempts to require exact match if exists', function () {
    if (process.platform === 'win32') {
      this.skip();
    }

    Object.defineProperty(process, 'platform', { value: 'linux' });
    Object.defineProperty(process, 'arch', { value: 'x64' });
    process.env.ROS_DISTRO = 'humble';

    // Stub fs.existsSync to return true
    const existsSync = sandbox.stub(fs, 'existsSync').returns(true);
    assert.strictEqual(loader.customFallbackLoader(), null);

    // Verify it checked the exact-match prebuild candidate. The loader may
    // probe additional paths (e.g. the build/Release mirror target), so find
    // the prebuild candidate check rather than relying on call order.
    assert.ok(existsSync.called);
    const args = existsSync
      .getCalls()
      .map((call) => call.args[0])
      .find((p) => p.includes('prebuilds') && p.includes('rclnodejs.node'));
    assert.ok(args, 'expected a prebuild candidate path to be checked');
    assert.ok(args.includes('humble'));
    assert.ok(args.includes('-node-'));
  });

  it('customFallbackLoader includes electron runtime in exact match path', function () {
    if (process.platform === 'win32') {
      this.skip();
    }

    Object.defineProperty(process, 'platform', { value: 'linux' });
    Object.defineProperty(process, 'arch', { value: 'x64' });
    process.env.ROS_DISTRO = 'humble';
    process.env.npm_config_runtime = 'electron';

    const existsSync = sandbox.stub(fs, 'existsSync').returns(true);
    assert.strictEqual(loader.customFallbackLoader(), null);

    assert.ok(existsSync.called);
    const args = existsSync
      .getCalls()
      .map((call) => call.args[0])
      .find((p) => p.includes('prebuilds') && p.includes('rclnodejs.node'));
    assert.ok(args, 'expected a prebuild candidate path to be checked');
    assert.ok(args.includes('humble'));
    assert.ok(args.includes('-electron-'));
  });

  it('loadNativeAddon force build triggers rebuild', async function () {
    process.env.RCLNODEJS_FORCE_BUILD = '1';
    let built = false;
    const addon = {};
    const expected = path.join(
      packageRoot,
      'build',
      'Release',
      'rclnodejs.node'
    );
    stubLocalAddons(
      (candidate) => candidate === expected,
      () => {
        assert.strictEqual(built, true);
        return addon;
      }
    );
    const execSync = sandbox.stub(child_process, 'execSync').callsFake(() => {
      built = true;
    });
    const loaded = await import('../lib/native_loader.js?forcebuild=1');

    assert.strictEqual(loaded.default, addon);
    assert.strictEqual(addon.path, expected);
    assert.ok(execSync.calledOnce);
    assert.strictEqual(execSync.firstCall.args[0], 'npm run rebuild');
  });
});
