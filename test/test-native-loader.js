import assert from 'assert';
import sinon from 'sinon';

import fs from 'fs';
import child_process from 'child_process';

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
  });

  afterEach(function () {
    sandbox.restore();
    Object.defineProperty(process, 'platform', { value: originalPlatform });
    Object.defineProperty(process, 'arch', { value: originalArch });
    process.env = originalEnv;
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

    // Verify it checked for the file
    assert.ok(existsSync.called);
    const args = existsSync.lastCall.args[0];
    assert.ok(args.includes('humble'));
    assert.ok(args.includes('-node-'));
    assert.ok(args.includes('rclnodejs.node'));
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
    const args = existsSync.lastCall.args[0];
    assert.ok(args.includes('humble'));
    assert.ok(args.includes('-electron-'));
    assert.ok(args.includes('rclnodejs.node'));
  });

  it('loadNativeAddon force build triggers rebuild', async function () {
    process.env.RCLNODEJS_FORCE_BUILD = '1';
    const execSync = sandbox.stub(child_process, 'execSync');

    // Re-evaluate native_loader via a cache-busting dynamic import to trigger
    // loadNativeAddon with RCLNODEJS_FORCE_BUILD. execSync is stubbed so no
    // real rebuild (which deletes generated/) occurs.
    try {
      await import('../lib/native_loader.js?forcebuild=1');
    } catch (e) {
      // Ignore if loading binding fails, as long as execSync was called
    }

    assert.ok(execSync.calledOnce);
    assert.match(execSync.firstCall.args[0], /npm run rebuild/);
  });
});
