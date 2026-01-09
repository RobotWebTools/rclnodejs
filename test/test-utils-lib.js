'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const os = require('os');
const sinon = require('sinon');
const utils = require('../lib/utils.js');

describe('Utils testing', function () {
  let tmpDir;
  const sandbox = sinon.createSandbox();

  beforeEach(function () {
    tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), 'rclnodejs-test-utils-'));
  });

  afterEach(function () {
    if (tmpDir && fs.existsSync(tmpDir)) {
      fs.rmSync(tmpDir, { recursive: true, force: true });
    }
    sandbox.restore();
  });

  it('should verify pathExists works correctly', async function () {
    const file = path.join(tmpDir, 'test-file.txt');
    fs.writeFileSync(file, 'content');

    assert.strictEqual(await utils.pathExists(file), true);
    assert.strictEqual(
      await utils.pathExists(path.join(tmpDir, 'non-existent')),
      false
    );
  });

  it('should valid ensureDir works correctly', async function () {
    const dir = path.join(tmpDir, 'nested/dir');
    await utils.ensureDir(dir);

    assert.ok(fs.existsSync(dir));
    const stat = fs.statSync(dir);
    assert.ok(stat.isDirectory());

    // Should not throw if it exists
    await utils.ensureDir(dir);
  });

  it('should valid ensureDirSync works correctly', function () {
    const dir = path.join(tmpDir, 'nested/sync/dir');
    utils.ensureDirSync(dir);

    assert.ok(fs.existsSync(dir));
    const stat = fs.statSync(dir);
    assert.ok(stat.isDirectory());

    // Should not throw if it exists
    utils.ensureDirSync(dir);
  });

  it('should valid emptyDir works correctly', async function () {
    const dir = path.join(tmpDir, 'cleanup');
    fs.mkdirSync(dir);
    fs.writeFileSync(path.join(dir, 'file1'), '1');
    fs.mkdirSync(path.join(dir, 'subdir'));
    fs.writeFileSync(path.join(dir, 'subdir/file2'), '2');

    await utils.emptyDir(dir);

    assert.ok(fs.existsSync(dir));
    const files = fs.readdirSync(dir);
    assert.strictEqual(files.length, 0);
  });

  it('should handle emptyDir on non-existent directory', async function () {
    const dir = path.join(tmpDir, 'non-existent-dir');
    await utils.emptyDir(dir);
    assert.ok(!fs.existsSync(dir));
  });

  it('should valid copy works correctly', async function () {
    const src = path.join(tmpDir, 'src');
    const dest = path.join(tmpDir, 'dest');

    await utils.ensureDir(src);
    fs.writeFileSync(path.join(src, 'file.txt'), 'hello');

    await utils.copy(src, dest);

    assert.ok(fs.existsSync(path.join(dest, 'file.txt')));
    assert.strictEqual(
      fs.readFileSync(path.join(dest, 'file.txt'), 'utf8'),
      'hello'
    );
  });

  it('should verify file operation wrappers', async function () {
    const file = path.join(tmpDir, 'wrap.txt');
    await utils.writeFile(file, 'data');
    assert.ok(fs.existsSync(file));
    assert.strictEqual(fs.readFileSync(file, 'utf8'), 'data');

    utils.removeSync(file);
    assert.ok(!fs.existsSync(file));

    await utils.mkdir(path.join(tmpDir, 'wrap-dir'));
    assert.ok(fs.existsSync(path.join(tmpDir, 'wrap-dir')));

    await utils.remove(path.join(tmpDir, 'wrap-dir'));
    assert.ok(!fs.existsSync(path.join(tmpDir, 'wrap-dir')));
  });

  it('should verify readJsonSync', function () {
    const file = path.join(tmpDir, 'data.json');
    fs.writeFileSync(file, '{"a":1}');
    const data = utils.readJsonSync(file);
    assert.deepStrictEqual(data, { a: 1 });
  });

  describe('Other utils testing', function () {
    it('should detect ubuntu codename correctly', function () {
      const originalPlatform = process.platform;
      Object.defineProperty(process, 'platform', { value: 'linux' });

      sandbox
        .stub(fs, 'readFileSync')
        .withArgs('/etc/os-release', 'utf8')
        .returns('VERSION_CODENAME=noble\nNAME="Ubuntu"');

      const codename = utils.detectUbuntuCodename();
      assert.strictEqual(codename, 'noble');

      Object.defineProperty(process, 'platform', { value: originalPlatform });
    });

    it('should return null for codename if not linux', function () {
      const originalPlatform = process.platform;
      Object.defineProperty(process, 'platform', { value: 'win32' });

      const codename = utils.detectUbuntuCodename();
      assert.strictEqual(codename, null);

      Object.defineProperty(process, 'platform', { value: originalPlatform });
    });

    it('should return null for codename if file read fails', function () {
      const originalPlatform = process.platform;
      Object.defineProperty(process, 'platform', { value: 'linux' });

      sandbox.stub(fs, 'readFileSync').throws(new Error('no file'));

      const codename = utils.detectUbuntuCodename();
      assert.strictEqual(codename, null);

      Object.defineProperty(process, 'platform', { value: originalPlatform });
    });

    it('should test normalizeNodeName', function () {
      assert.strictEqual(utils.normalizeNodeName('/node'), 'node');
      assert.strictEqual(utils.normalizeNodeName('node'), 'node');
      assert.strictEqual(utils.normalizeNodeName('/ns/node'), 'ns/node');
    });

    it('should test isClose', function () {
      assert.strictEqual(utils.isClose(1.0, 1.0), true);
      assert.strictEqual(utils.isClose(1.0, 1.0000000001), true); // 1e-10 diff
      assert.strictEqual(utils.isClose(1.0, 1.1), false);
      assert.strictEqual(utils.isClose(0, 0), true);
      // Implementation check: utils.js returns false if not finite, UNLESS they are strictly equal
      assert.strictEqual(utils.isClose(Infinity, Infinity), true);
      assert.strictEqual(utils.isClose(1, Infinity), false);
    });

    it('should test compareVersions', function () {
      assert.strictEqual(utils.compareVersions('1.2.3', '1.2.3', '=='), true);
      assert.strictEqual(utils.compareVersions('1.2.3', '1.2.4', '<'), true);
      assert.strictEqual(utils.compareVersions('1.3', '1.2.4', '>'), true);
      assert.strictEqual(utils.compareVersions('1.2.3', '1.2', '>'), true);
      assert.throws(() => utils.compareVersions('1.0', '1.0', 'badop'));
    });
  });
});
