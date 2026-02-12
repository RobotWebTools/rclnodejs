'use strict';

const assert = require('assert');
const sinon = require('sinon');

const fs = require('fs');
const child_process = require('child_process');

// Require the module once without clearing cache to avoid triggering
// loadNativeAddon() repeatedly. The TestHelpers functions read process
// state at call time, so they can be tested with platform/env changes
// without re-requiring the module.
// NOTE: The module may have been loaded by other tests before NODE_ENV was set,
// so TestHelpers might not be present. We clear cache and re-require with
// NODE_ENV='test' to ensure TestHelpers is exported.
const nativeLoaderPath = require.resolve('../lib/native_loader.js');
if (
  !require.cache[nativeLoaderPath] ||
  !require.cache[nativeLoaderPath].exports.TestHelpers
) {
  delete require.cache[nativeLoaderPath];
  process.env.NODE_ENV = 'test';
}
const nativeLoader = require('../lib/native_loader.js');

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
    assert.ok(args.includes('rclnodejs.node'));
  });

  it('loadNativeAddon force build triggers rebuild', function () {
    process.env.RCLNODEJS_FORCE_BUILD = '1';
    const execSync = sandbox.stub(child_process, 'execSync');

    // Clear cache and re-require to trigger loadNativeAddon with RCLNODEJS_FORCE_BUILD.
    // execSync is stubbed so no real rebuild (which deletes generated/) occurs.
    delete require.cache[require.resolve('../lib/native_loader.js')];

    try {
      require('../lib/native_loader.js');
    } catch (e) {
      // Ignore if loading binding fails, as long as execSync was called
    }

    assert.ok(execSync.calledOnce);
    assert.match(execSync.firstCall.args[0], /npm run rebuild/);
  });
});
