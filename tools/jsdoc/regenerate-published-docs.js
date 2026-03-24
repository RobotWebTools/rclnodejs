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
const defaultOutputRoot = path.join(repoRoot, 'build', 'gh-pages-docs');
const semverDirectoryPattern = /^\d+\.\d+\.\d+$/;
const sharedAssetDirectoryName = '_static';
const sharedAssetFolders = ['fonts', 'scripts', 'styles'];

function parseArgs(argv) {
  const options = {
    branch: 'gh-pages',
    outputRoot: defaultOutputRoot,
    versions: null,
    preservePublished: false,
    fullRebuild: false,
    keepWorktrees: false,
  };

  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];

    if (arg === '--branch') {
      options.branch = argv[index + 1];
      index += 1;
    } else if (arg === '--out') {
      options.outputRoot = path.resolve(argv[index + 1]);
      index += 1;
    } else if (arg === '--versions') {
      options.versions = argv[index + 1]
        .split(',')
        .map((value) => value.trim())
        .filter(Boolean);
      index += 1;
    } else if (arg === '--preserve-published') {
      options.preservePublished = true;
    } else if (arg === '--full-rebuild') {
      options.fullRebuild = true;
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

function getPublishedVersions(branch) {
  const output = runCommand('git', [
    'ls-tree',
    '--name-only',
    `${branch}:docs`,
  ]);

  return output
    .split(/\r?\n/)
    .map((entry) => entry.trim())
    .filter((entry) => semverDirectoryPattern.test(entry))
    .sort(compareVersionsAsc);
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

function writeBranchFile(branch, sourcePath, destinationPath) {
  try {
    const contents = runCommand('git', ['show', `${branch}:${sourcePath}`]);
    fs.mkdirSync(path.dirname(destinationPath), { recursive: true });
    fs.writeFileSync(destinationPath, contents, 'utf8');
  } catch (error) {
    if (sourcePath === '.nojekyll') {
      fs.writeFileSync(destinationPath, '', 'utf8');
      return;
    }

    throw error;
  }
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

function copyIfExists(sourcePath, destinationPath) {
  if (!fs.existsSync(sourcePath)) {
    return;
  }

  fs.mkdirSync(path.dirname(destinationPath), { recursive: true });
  fs.cpSync(sourcePath, destinationPath, { recursive: true, force: true });
}

function copyDirectoryContents(sourceDir, destinationDir) {
  if (!fs.existsSync(sourceDir)) {
    return;
  }

  fs.mkdirSync(destinationDir, { recursive: true });

  fs.readdirSync(sourceDir, { withFileTypes: true }).forEach((entry) => {
    fs.cpSync(
      path.join(sourceDir, entry.name),
      path.join(destinationDir, entry.name),
      { recursive: true, force: true }
    );
  });
}

function copyPublishedSnapshot(branch, outputRoot, tempRoot, keepWorktrees) {
  const worktreePath = addWorktree(branch, tempRoot, `branch-${branch}`);
  const rootFiles = ['.nojekyll', 'README.md', '_index-package.json'];

  try {
    copyDirectoryContents(
      path.join(worktreePath, 'docs'),
      path.join(outputRoot, 'docs')
    );

    rootFiles.forEach((fileName) => {
      copyIfExists(
        path.join(worktreePath, fileName),
        path.join(outputRoot, fileName)
      );
    });
  } finally {
    removeWorktree(worktreePath, keepWorktrees);
  }
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

function ensureSharedAssetsForVersion(outputRoot, version) {
  const docsRoot = path.join(outputRoot, 'docs');
  const versionRoot = path.join(docsRoot, version);
  const sharedRoot = path.join(docsRoot, sharedAssetDirectoryName);

  if (!fs.existsSync(versionRoot)) {
    return;
  }

  fs.mkdirSync(sharedRoot, { recursive: true });

  sharedAssetFolders.forEach((folderName) => {
    const sourcePath = path.join(versionRoot, folderName);

    if (!fs.existsSync(sourcePath)) {
      return;
    }

    fs.cpSync(sourcePath, path.join(sharedRoot, folderName), {
      recursive: true,
      force: true,
    });
  });

  rewriteVersionHtmlToSharedAssets(versionRoot);

  sharedAssetFolders.forEach((folderName) => {
    fs.rmSync(path.join(versionRoot, folderName), {
      recursive: true,
      force: true,
    });
  });
}

function main() {
  const options = parseArgs(process.argv.slice(2));

  if (options.preservePublished && options.fullRebuild) {
    throw new Error(
      'Use either --preserve-published or --full-rebuild, not both.'
    );
  }

  const publishedVersions = options.versions
    ? options.versions
    : getPublishedVersions(options.branch);
  const currentVersion = getCurrentWorkspaceVersion();
  const versions = Array.from(
    new Set(publishedVersions.concat(currentVersion))
  ).sort(compareVersionsAsc);

  if (!versions.length) {
    throw new Error(`No published versions found in ${options.branch}:docs`);
  }

  assertTagsExist(
    publishedVersions.filter((version) => version !== currentVersion)
  );
  ensureCleanOutput(options.outputRoot);

  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), 'rclnodejs-jsdoc-'));
  const latestVersion = versions[versions.length - 1];
  let latestPackageJsonPath = null;

  try {
    if (options.preservePublished) {
      copyPublishedSnapshot(
        options.branch,
        options.outputRoot,
        tempRoot,
        options.keepWorktrees
      );
      console.log(
        `Preserved ${publishedVersions.length} published versions from ${options.branch}.`
      );
    } else {
      writeBranchFile(
        options.branch,
        '.nojekyll',
        path.join(options.outputRoot, '.nojekyll')
      );
      writeBranchFile(
        options.branch,
        'README.md',
        path.join(options.outputRoot, 'README.md')
      );
    }

    console.log(`Regenerating ${versions.length} documentation versions.`);

    if (!options.preservePublished || options.fullRebuild) {
      publishedVersions.forEach((version) => {
        if (version === currentVersion) {
          return;
        }

        console.log(`- ${version}`);

        const buildResult = buildVersionDocs(
          version,
          path.join(options.outputRoot, 'docs'),
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
    }

    if (options.preservePublished && !options.fullRebuild) {
      const existingIndexPackagePath = path.join(
        options.outputRoot,
        '_index-package.json'
      );

      if (fs.existsSync(existingIndexPackagePath)) {
        latestPackageJsonPath = existingIndexPackagePath;
      }
    }

    console.log(`- ${currentVersion} (current workspace)`);

    const buildResult = buildDocsFromSourceRoot(
      repoRoot,
      path.join(options.outputRoot, 'docs')
    );

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

    if (options.preservePublished && !options.fullRebuild) {
      ensureSharedAssetsForVersion(options.outputRoot, currentVersion);
    } else {
      hoistSharedAssets(options.outputRoot);
    }

    removeTemporaryPublishArtifacts(options.outputRoot);

    console.log(`Published docs tree ready at ${options.outputRoot}`);
  } finally {
    if (!options.keepWorktrees) {
      fs.rmSync(tempRoot, { recursive: true, force: true });
    }
  }
}

main();
