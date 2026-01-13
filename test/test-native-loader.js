'use strict';

const assert = require('assert');
const sinon = require('sinon');

const fs = require('fs');
const child_process = require('child_process');

describe('NativeLoader testing', function () {
  const sandbox = sinon.createSandbox();
  let originalPlatform;
  let originalArch;
  let originalEnv;

  beforeEach(function () {
    originalPlatform = process.platform;
    originalArch = process.arch;
    originalEnv = { ...process.env };
    process.env.NODE_ENV = 'test';

    // Clear cache to reload module
    delete require.cache[require.resolve('../lib/native_loader.js')];
  });

  afterEach(function () {
    sandbox.restore();
    Object.defineProperty(process, 'platform', { value: originalPlatform });
    Object.defineProperty(process, 'arch', { value: originalArch });
    process.env = originalEnv;
  });

  function getLoader() {
    return require('../lib/native_loader.js').TestHelpers;
  }

  it('customFallbackLoader returns null on non-linux', function () {
    Object.defineProperty(process, 'platform', { value: 'win32' });
    const loader = getLoader();
    assert.strictEqual(loader.customFallbackLoader(), null);
  });

  it('customFallbackLoader returns null if env info missing', function () {
    Object.defineProperty(process, 'platform', { value: 'linux' });
    process.env.ROS_DISTRO = '';
    const loader = getLoader();
    assert.strictEqual(loader.customFallbackLoader(), null);
  });

  it('customFallbackLoader returns null if prebuild dir not found', function () {
    Object.defineProperty(process, 'platform', { value: 'linux' });
    Object.defineProperty(process, 'arch', { value: 'x64' });
    process.env.ROS_DISTRO = 'humble';

    sandbox.stub(fs, 'existsSync').returns(false);

    const loader = getLoader();
    assert.strictEqual(loader.customFallbackLoader(), null);
  });

  it('customFallbackLoader attempts to require exact match if exists', function () {
    Object.defineProperty(process, 'platform', { value: 'linux' });
    Object.defineProperty(process, 'arch', { value: 'x64' });
    process.env.ROS_DISTRO = 'humble';

    // Stub fs.existsSync to return true
    const existsSync = sandbox.stub(fs, 'existsSync').returns(true);

    const loader = getLoader();
    assert.strictEqual(loader.customFallbackLoader(), null);

    // Verify it checked for the file
    assert.ok(existsSync.called);
    const args = existsSync.lastCall.args[0];
    assert.ok(args.includes('humble'));
    assert.ok(args.includes('rclnodejs.node'));
  });

  it('loadNativeAddon force build triggers rebuild', function () {
    process.env.RCLNODEJS_FORCE_BUILD = '1';
    const execSync = sandbox.stub(child_process, 'execSync');

    // We expect it to try loading bindings after build
    // Since we don't mock bindings, it will load the real one (if present) or fail.
    // If it loads real one, test passes.
    // If it fails, loadNativeAddon throws. We might need to handle that.
    // But usually in dev env, the binding exists.

    try {
      getLoader();
      // Wait, getLoader requires the file.
      // The file calls loadNativeAddon() immediately.
      // So valid test is just requiring the file.
    } catch (e) {
      // Ignore if loading binding fails, as long as execSync was called
    }

    assert.ok(execSync.calledOnce);
    assert.match(execSync.firstCall.args[0], /npm run rebuild/);
  });
});
