// Copyright (c) 2026 The Robot Web Tools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const childProcess = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

const repoRoot = path.resolve(__dirname, '../..');
const buildIndexScript = path.join(__dirname, 'build-index.js');
const defaultOutputRoot = path.join(repoRoot, 'build', 'published-docs');
const semverDirectoryPattern = /^\d+\.\d+\.\d+$/;
const sharedAssetDirectoryName = '_static';
const sharedAssetFolders = ['fonts', 'scripts', 'styles'];

const defaultManifestPath = path.join(__dirname, 'published-versions.json');

function parseArgs(argv) {
  const options = {
    outputRoot: defaultOutputRoot,
    manifestPath: defaultManifestPath,
    keepWorktrees: false,
  };

  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];

    if (arg === '--out') {
      options.outputRoot = path.resolve(argv[index + 1]);
      index += 1;
    } else if (arg === '--manifest') {
      options.manifestPath = path.resolve(argv[index + 1]);
      index += 1;
    } else if (arg === '--keep-worktrees') {
      options.keepWorktrees = true;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }

  return options;
}

function runCommand(command, args, options = {}) {
  try {
    return childProcess.execFileSync(command, args, {
      cwd: options.cwd || repoRoot,
      encoding: 'utf8',
      env: options.env || process.env,
      stdio: ['ignore', 'pipe', 'pipe'],
    });
  } catch (error) {
    const stderr = error.stderr ? String(error.stderr).trim() : '';
    const stdout = error.stdout ? String(error.stdout).trim() : '';
    const details = [stderr, stdout].filter(Boolean).join('\n');

    throw new Error(
      details
        ? `${command} ${args.join(' ')}\n${details}`
        : `${command} ${args.join(' ')} failed`
    );
  }
}

function compareVersionsAsc(left, right) {
  const leftParts = left.split('.').map(Number);
  const rightParts = right.split('.').map(Number);
  const maxLength = Math.max(leftParts.length, rightParts.length);

  for (let index = 0; index < maxLength; index += 1) {
    const leftValue = leftParts[index] || 0;
    const rightValue = rightParts[index] || 0;

    if (leftValue !== rightValue) {
      return leftValue - rightValue;
    }
  }

  return 0;
}

function readManifestVersions(manifestPath) {
  let parsed;

  try {
    parsed = JSON.parse(fs.readFileSync(manifestPath, 'utf8'));
  } catch (error) {
    throw new Error(
      `Unable to read versions manifest ${manifestPath}: ${error.message}`
    );
  }

  const list = Array.isArray(parsed) ? parsed : parsed.versions;

  if (!Array.isArray(list)) {
    throw new Error(
      `Manifest ${manifestPath} must be an array or contain a "versions" array.`
    );
  }

  const versions = list.map((value) => String(value).trim()).filter(Boolean);
  const invalid = versions.filter(
    (value) => !semverDirectoryPattern.test(value)
  );

  if (invalid.length) {
    throw new Error(
      `Manifest ${manifestPath} contains non-release versions: ${invalid.join(
        ', '
      )}`
    );
  }

  return Array.from(new Set(versions)).sort(compareVersionsAsc);
}

function getTaggedReleaseVersions() {
  return runCommand('git', ['tag', '--list'])
    .split(/\r?\n/)
    .map((entry) => entry.trim())
    .filter((entry) => semverDirectoryPattern.test(entry))
    .sort(compareVersionsAsc);
}

function getCurrentWorkspaceVersion() {
  const packageInfo = JSON.parse(
    fs.readFileSync(path.join(repoRoot, 'package.json'), 'utf8')
  );

  if (!semverDirectoryPattern.test(packageInfo.version)) {
    throw new Error(
      `Current package.json version is not a release semver: ${packageInfo.version}`
    );
  }

  return packageInfo.version;
}

function assertTagsExist(versions) {
  const tags = new Set(getTaggedReleaseVersions());

  const missing = versions.filter((version) => !tags.has(version));

  if (missing.length) {
    throw new Error(
      `Missing Git tags for requested versions: ${missing.join(', ')}`
    );
  }
}

function ensureCleanOutput(outputRoot) {
  fs.rmSync(outputRoot, { recursive: true, force: true });
  fs.mkdirSync(path.join(outputRoot, 'docs'), { recursive: true });
}

function getGeneratedVersions(outputRoot) {
  const docsRoot = path.join(outputRoot, 'docs');

  if (!fs.existsSync(docsRoot)) {
    return [];
  }

  return fs
    .readdirSync(docsRoot, { withFileTypes: true })
    .filter(
      (entry) => entry.isDirectory() && semverDirectoryPattern.test(entry.name)
    )
    .map((entry) => entry.name)
    .sort(compareVersionsAsc);
}

function appendJsFiles(directoryPath, inputs) {
  if (!fs.existsSync(directoryPath)) {
    return;
  }

  fs.readdirSync(directoryPath, { withFileTypes: true })
    .filter((entry) => entry.isFile() && entry.name.endsWith('.js'))
    .sort((left, right) => left.name.localeCompare(right.name))
    .forEach((entry) => {
      inputs.push(path.join(directoryPath, entry.name));
    });
}

function getJsdocInputs(sourceRoot) {
  const inputs = [];
  const indexPath = path.join(sourceRoot, 'index.js');

  if (fs.existsSync(indexPath)) {
    inputs.push(indexPath);
  }

  appendJsFiles(path.join(sourceRoot, 'lib'), inputs);
  appendJsFiles(path.join(sourceRoot, 'lib', 'action'), inputs);

  if (!inputs.length) {
    throw new Error(`No JSDoc input files found for ${sourceRoot}`);
  }

  return inputs;
}

function sanitizePathSegment(value) {
  return value.replace(/[^a-zA-Z0-9._-]/g, '_');
}

function addWorktree(ref, tempRoot, directoryName = ref) {
  const worktreePath = path.join(tempRoot, sanitizePathSegment(directoryName));

  runCommand('git', [
    'worktree',
    'add',
    '--detach',
    '--force',
    worktreePath,
    ref,
  ]);

  return worktreePath;
}

function removeWorktree(worktreePath, keepWorktrees) {
  if (keepWorktrees) {
    return;
  }

  try {
    runCommand('git', ['worktree', 'remove', '--force', worktreePath]);
  } catch {
    fs.rmSync(worktreePath, { recursive: true, force: true });
  }
}

function writeIndexPackage(packageJsonPath, outputRoot) {
  const packageInfo = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));
  const trimmedPackageInfo = {
    name: packageInfo.name,
    version: packageInfo.version,
    repository: packageInfo.repository,
  };
  const destinationPath = path.join(outputRoot, '_index-package.json');

  fs.writeFileSync(
    destinationPath,
    `${JSON.stringify(trimmedPackageInfo, null, 2)}\n`,
    'utf8'
  );

  return destinationPath;
}

function buildVersionDocs(version, outputDocsRoot, tempRoot, keepWorktrees) {
  const worktreePath = addWorktree(version, tempRoot, version);

  try {
    const inputs = getJsdocInputs(worktreePath);

    runCommand(
      'npx',
      [
        'jsdoc',
        '--package',
        path.join(worktreePath, 'package.json'),
        ...inputs,
        '-t',
        __dirname,
        '-d',
        outputDocsRoot,
      ],
      { cwd: repoRoot }
    );

    return {
      worktreePath,
      packageJsonPath: path.join(worktreePath, 'package.json'),
    };
  } catch (error) {
    removeWorktree(worktreePath, keepWorktrees);
    throw error;
  }
}

function buildDocsFromSourceRoot(sourceRoot, outputDocsRoot) {
  const inputs = getJsdocInputs(sourceRoot);

  runCommand(
    'npx',
    [
      'jsdoc',
      '--package',
      path.join(sourceRoot, 'package.json'),
      ...inputs,
      '-t',
      __dirname,
      '-d',
      outputDocsRoot,
    ],
    { cwd: repoRoot }
  );

  return {
    packageJsonPath: path.join(sourceRoot, 'package.json'),
  };
}

function buildDocsIndex(outputRoot, packageJsonPath) {
  runCommand('node', [buildIndexScript], {
    cwd: repoRoot,
    env: {
      ...process.env,
      RCLNODEJS_DOCS_ROOT: path.join(outputRoot, 'docs'),
      RCLNODEJS_DOCS_INDEX_PATH: path.join(outputRoot, 'docs', 'index.html'),
      RCLNODEJS_LOCAL_INDEX_PATH: '',
      RCLNODEJS_PACKAGE_JSON_PATH: packageJsonPath,
      RCLNODEJS_GIT_CWD: repoRoot,
    },
  });
}

function removeTemporaryPublishArtifacts(outputRoot) {
  ['README.md', '_index-package.json'].forEach((fileName) => {
    fs.rmSync(path.join(outputRoot, fileName), { force: true });
  });
}

function rewriteVersionHtmlToSharedAssets(versionRoot) {
  fs.readdirSync(versionRoot, { withFileTypes: true })
    .filter((entry) => entry.isFile() && entry.name.endsWith('.html'))
    .forEach((entry) => {
      const filePath = path.join(versionRoot, entry.name);
      const original = fs.readFileSync(filePath, 'utf8');
      const rewritten = original.replace(
        /(href|src)="(styles|scripts)\//g,
        '$1="../_static/$2/'
      );

      if (rewritten !== original) {
        fs.writeFileSync(filePath, rewritten, 'utf8');
      }
    });
}

function hoistSharedAssets(outputRoot) {
  const docsRoot = path.join(outputRoot, 'docs');
  const versions = getGeneratedVersions(outputRoot);

  if (!versions.length) {
    return;
  }

  const sourceVersion = versions[versions.length - 1];
  const sourceVersionRoot = path.join(docsRoot, sourceVersion);
  const sharedRoot = path.join(docsRoot, sharedAssetDirectoryName);

  fs.mkdirSync(sharedRoot, { recursive: true });

  sharedAssetFolders.forEach((folderName) => {
    fs.cpSync(
      path.join(sourceVersionRoot, folderName),
      path.join(sharedRoot, folderName),
      { recursive: true, force: true }
    );
  });

  versions.forEach((version) => {
    const versionRoot = path.join(docsRoot, version);

    rewriteVersionHtmlToSharedAssets(versionRoot);

    sharedAssetFolders.forEach((folderName) => {
      fs.rmSync(path.join(versionRoot, folderName), {
        recursive: true,
        force: true,
      });
    });
  });
}

function main() {
  const options = parseArgs(process.argv.slice(2));

  const publishedVersions = readManifestVersions(options.manifestPath);
  const currentVersion = getCurrentWorkspaceVersion();
  const versions = Array.from(
    new Set(publishedVersions.concat(currentVersion))
  ).sort(compareVersionsAsc);
  const latestVersion = versions[versions.length - 1];

  // The staged site is a pure function of the manifest and the Git tags: every
  // published version is rebuilt from its tag and the in-development version
  // from the current workspace. There is no published-branch state to drift, so
  // a fresh build always reproduces the full site.
  assertTagsExist(
    publishedVersions.filter((version) => version !== currentVersion)
  );
  ensureCleanOutput(options.outputRoot);
  fs.writeFileSync(path.join(options.outputRoot, '.nojekyll'), '', 'utf8');

  const docsRoot = path.join(options.outputRoot, 'docs');
  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), 'rclnodejs-jsdoc-'));
  let latestPackageJsonPath = null;

  try {
    console.log(`Building ${versions.length} documentation versions.`);

    publishedVersions.forEach((version) => {
      if (version === currentVersion) {
        return;
      }

      console.log(`- ${version} (from tag)`);

      const buildResult = buildVersionDocs(
        version,
        docsRoot,
        tempRoot,
        options.keepWorktrees
      );

      if (version === latestVersion) {
        latestPackageJsonPath = writeIndexPackage(
          buildResult.packageJsonPath,
          options.outputRoot
        );
      }

      removeWorktree(buildResult.worktreePath, options.keepWorktrees);
    });

    console.log(`- ${currentVersion} (current workspace)`);

    const buildResult = buildDocsFromSourceRoot(repoRoot, docsRoot);

    if (currentVersion === latestVersion) {
      latestPackageJsonPath = writeIndexPackage(
        buildResult.packageJsonPath,
        options.outputRoot
      );
    }

    buildDocsIndex(
      options.outputRoot,
      latestPackageJsonPath || path.join(repoRoot, 'package.json')
    );
    hoistSharedAssets(options.outputRoot);
    removeTemporaryPublishArtifacts(options.outputRoot);

    console.log(`Published docs tree ready at ${options.outputRoot}`);
  } finally {
    if (!options.keepWorktrees) {
      fs.rmSync(tempRoot, { recursive: true, force: true });
    }
  }
}

main();
