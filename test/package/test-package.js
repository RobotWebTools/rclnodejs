// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  detectPrebuildRuntime,
  getTaggedPrebuildFilename,
} from '../../lib/prebuilds.js';
import { detectUbuntuCodename } from '../../lib/utils.js';

const root = fileURLToPath(new URL('../../', import.meta.url));

function run(command, args, cwd, env = process.env) {
  return execFileSync(command, args, {
    cwd,
    env,
    encoding: 'utf8',
    timeout: 120000,
    maxBuffer: 16 * 1024 * 1024,
    stdio: ['ignore', 'pipe', 'pipe'],
  });
}

function runNpm(args, cwd) {
  assert.ok(process.env.npm_execpath, 'Run package tests through npm');
  return run(process.execPath, [process.env.npm_execpath, ...args], cwd);
}

describe('Installed package compatibility', function () {
  this.timeout(120000);
  let directory;
  let packageRoot;
  let packedFiles;
  let prebuiltPath;
  let env;

  before(function () {
    assert.ok(process.env.ROS_DISTRO, 'Source ROS 2 before package tests');
    directory = fs.mkdtempSync(path.join(os.tmpdir(), 'rclnodejs-package-'));
    const packResult = JSON.parse(
      runNpm(
        ['pack', '--ignore-scripts', '--json', '--pack-destination', directory],
        root
      )
    );
    // npm 12 keys archive metadata by package name instead of returning an array.
    const archive = Array.isArray(packResult)
      ? packResult[0]
      : packResult?.rclnodejs;
    assert.ok(
      typeof archive?.filename === 'string' && Array.isArray(archive.files),
      'npm pack returned invalid archive metadata for rclnodejs'
    );
    packedFiles = new Set(archive.files.map((file) => file.path));
    fs.writeFileSync(
      path.join(directory, 'package.json'),
      JSON.stringify({
        name: 'rclnodejs-package-consumer',
        private: true,
        type: 'module',
        dependencies: { rclnodejs: `file:./${archive.filename}` },
      })
    );
    runNpm(
      [
        'install',
        '--ignore-scripts',
        '--no-package-lock',
        '--no-audit',
        '--no-fund',
      ],
      directory
    );
    packageRoot = path.join(directory, 'node_modules', 'rclnodejs');
    fs.copyFileSync(
      fileURLToPath(new URL('./consumer.mjs', import.meta.url)),
      path.join(directory, 'consumer.mjs')
    );

    const ubuntuCodename = detectUbuntuCodename();
    if (process.platform === 'linux' && ubuntuCodename) {
      prebuiltPath = path.join(
        'prebuilds',
        `linux-${process.arch}`,
        getTaggedPrebuildFilename({
          rosDistro: process.env.ROS_DISTRO,
          ubuntuCodename,
          arch: process.arch,
          runtime: detectPrebuildRuntime(),
        })
      );
    }
    const nativePath =
      prebuiltPath && fs.existsSync(path.join(root, prebuiltPath))
        ? path.join(root, prebuiltPath)
        : path.join(root, 'build', 'Release', 'rclnodejs.node');
    assert.ok(
      fs.existsSync(nativePath),
      'Build the native addon before package tests'
    );
    const localBuild = path.join(packageRoot, 'build', 'Release');
    fs.mkdirSync(localBuild, { recursive: true });
    fs.copyFileSync(nativePath, path.join(localBuild, 'rclnodejs.node'));
    if (prebuiltPath) {
      fs.mkdirSync(path.dirname(path.join(packageRoot, prebuiltPath)), {
        recursive: true,
      });
      fs.copyFileSync(nativePath, path.join(packageRoot, prebuiltPath));
    }
    const overlay = path.join(directory, 'overlay');
    const index = path.join(
      overlay,
      'share',
      'ament_index',
      'resource_index',
      'rosidl_interfaces'
    );
    const messages = path.join(overlay, 'share', 'tsdown_test_msgs', 'msg');
    fs.mkdirSync(index, { recursive: true });
    fs.mkdirSync(messages, { recursive: true });
    fs.writeFileSync(
      path.join(index, 'tsdown_test_msgs'),
      'msg/Envelope.msg\n'
    );
    fs.writeFileSync(
      path.join(messages, 'Envelope.msg'),
      'std_msgs/Header header\nstring label\nint64 serial\n'
    );
    env = {
      ...process.env,
      AMENT_PREFIX_PATH: [overlay, process.env.AMENT_PREFIX_PATH].join(
        path.delimiter
      ),
    };
    run(process.execPath, ['scripts/generate_messages.cjs'], packageRoot, env);
  });

  after(function () {
    if (directory) fs.rmSync(directory, { recursive: true, force: true });
  });

  it('ships the existing entry points, declarations and flat shared chunks', function () {
    for (const entry of ['index', 'web', 'server', 'rosocket']) {
      for (const extension of ['js', 'cjs']) {
        assert.ok(packedFiles.has(`dist/${entry}.${extension}`));
        assert.ok(packedFiles.has(`dist/${entry}.${extension}.map`));
      }
    }
    for (const declaration of [
      'types/index.d.ts',
      'web/index.d.ts',
      'lib/runtime/index.d.ts',
      'rosocket/index.d.ts',
      'rosocket/index.d.cts',
    ]) {
      assert.ok(packedFiles.has(declaration), declaration);
      assert.equal(
        fs.readFileSync(path.join(packageRoot, declaration), 'utf8'),
        fs.readFileSync(path.join(root, declaration), 'utf8')
      );
    }
    for (const filename of packedFiles) {
      if (filename.startsWith('dist/')) {
        assert.equal(filename.split('/').length, 2, filename);
        assert.ok(!filename.endsWith('.d.ts'), filename);
      }
    }
    assert.ok(!packedFiles.has('tsdown.config.js'));
    assert.ok(!packedFiles.has('tsup.config.js'));
    for (const file of ['rosidl_gen/index.cjs', 'rostsd_gen/index.cjs']) {
      assert.ok(packedFiles.has(file));
    }
  });

  it('preserves typed ESM and CommonJS consumer imports', function () {
    const source = `
import * as rclnodejs from 'rclnodejs';
import { connect } from 'rclnodejs/web';
import { createRuntime } from 'rclnodejs/web/server';
import { startRosocket } from 'rclnodejs/rosocket';

const node = rclnodejs.createNode('typed_package_consumer');
createRuntime({ node });
startRosocket({ node, port: 0 });
async function checkTypes() {
  const ros = await connect({ http: 'http://localhost:9001' });
  const goal = await ros.action<'example_interfaces/action/Fibonacci'>(
    '/action', { order: 3 }, {
      onFeedback: (message) => {
        const sequence: number[] = message.sequence;
        return sequence;
      },
    }
  );
  const sequence: number[] = (await goal.result).sequence;
  await goal.cancel();
  // @ts-expect-error Action goals must retain their generated field types.
  await ros.action<'example_interfaces/action/Fibonacci'>('/action', { order: '3' });
  return sequence;
}
`;
    for (const extension of ['mts', 'cts']) {
      fs.writeFileSync(path.join(directory, `consumer.${extension}`), source);
    }
    run(
      process.execPath,
      [
        path.join(root, 'node_modules/typescript/bin/tsc'),
        '--ignoreConfig',
        '--noEmit',
        '--strict',
        '--skipLibCheck',
        '--target',
        'es2022',
        '--module',
        'nodenext',
        '--types',
        'node',
        '--typeRoots',
        path.join(root, 'node_modules/@types'),
        'consumer.mts',
        'consumer.cts',
      ],
      directory
    );
  });

  for (const format of ['esm', 'cjs']) {
    it(`${format} loads a matching prebuild and runs pub/sub, services and actions`, function () {
      if (!prebuiltPath) this.skip();
      run(
        process.execPath,
        ['consumer.mjs', format, prebuiltPath],
        directory,
        env
      );
    });

    it(`${format} falls back to a local native build without prebuilds`, function () {
      const prebuilds = path.join(packageRoot, 'prebuilds');
      const backup = path.join(directory, 'prebuilds');
      const exists = fs.existsSync(prebuilds);
      if (exists) fs.renameSync(prebuilds, backup);
      try {
        run(
          process.execPath,
          ['consumer.mjs', format, 'build/Release/rclnodejs.node'],
          directory,
          env
        );
      } finally {
        if (exists) fs.renameSync(backup, prebuilds);
      }
    });
  }
});
