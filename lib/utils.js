// Copyright (c) 2025, The Robot Web Tools Contributors
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

const fs = require('fs');
const fsPromises = require('fs/promises');
const path = require('path');
const { ValidationError } = require('./errors.js');

/**
 * Ensure directory exists, create recursively if needed (async)
 * Replaces: fse.ensureDir() / fse.mkdirs()
 * @param {string} dirPath - Path to directory
 * @returns {Promise<void>}
 */
async function ensureDir(dirPath) {
  try {
    await fsPromises.mkdir(dirPath, { recursive: true });
  } catch (err) {
    // Ignore if directory already exists
    if (err.code !== 'EEXIST') throw err;
  }
}

/**
 * Ensure directory exists, create recursively if needed (sync)
 * Replaces: fse.mkdirSync()
 * @param {string} dirPath - Path to directory
 */
function ensureDirSync(dirPath) {
  try {
    fs.mkdirSync(dirPath, { recursive: true });
  } catch (err) {
    // Ignore if directory already exists
    if (err.code !== 'EEXIST') throw err;
  }
}

/**
 * Check if path exists (async)
 * Replaces: fse.exists()
 * @param {string} filePath - Path to check
 * @returns {Promise<boolean>}
 */
async function pathExists(filePath) {
  try {
    await fsPromises.access(filePath);
    return true;
  } catch {
    return false;
  }
}

/**
 * Empty a directory (remove all contents but keep the directory)
 * Replaces: fse.emptyDir()
 * @param {string} dirPath - Path to directory
 * @returns {Promise<void>}
 */
async function emptyDir(dirPath) {
  try {
    const files = await fsPromises.readdir(dirPath);
    await Promise.all(
      files.map((file) =>
        fsPromises.rm(path.join(dirPath, file), {
          recursive: true,
          force: true,
        })
      )
    );
  } catch (err) {
    // Ignore if directory doesn't exist
    if (err.code !== 'ENOENT') throw err;
  }
}

/**
 * Copy file or directory recursively
 * Replaces: fse.copy()
 * @param {string} src - Source path
 * @param {string} dest - Destination path
 * @param {object} options - Copy options
 * @returns {Promise<void>}
 */
async function copy(src, dest, options = {}) {
  const opts = {
    recursive: true,
    force: options.overwrite !== false,
    ...options,
  };
  await fsPromises.cp(src, dest, opts);
}

/**
 * Read and parse JSON file synchronously
 * Replaces: fse.readJsonSync()
 * @param {string} filePath - Path to JSON file
 * @param {object} options - Read options
 * @returns {any} Parsed JSON data
 */
function readJsonSync(filePath, options = {}) {
  const content = fs.readFileSync(filePath, options.encoding || 'utf8');
  return JSON.parse(content);
}

/**
 * Remove file or directory (async)
 * Replaces: fse.remove()
 * @param {string} filePath - Path to remove
 * @returns {Promise<void>}
 */
async function remove(filePath) {
  try {
    await fsPromises.rm(filePath, { recursive: true, force: true });
  } catch (err) {
    // Ignore if path doesn't exist
    if (err.code !== 'ENOENT') throw err;
  }
}

/**
 * Remove file or directory (sync)
 * Replaces: fse.removeSync()
 * @param {string} filePath - Path to remove
 */
function removeSync(filePath) {
  try {
    fs.rmSync(filePath, { recursive: true, force: true });
  } catch (err) {
    // Ignore if path doesn't exist
    if (err.code !== 'ENOENT') throw err;
  }
}

/**
 * Write file with content (async)
 * Replaces: fse.writeFile()
 * @param {string} filePath - Path to file
 * @param {string|Buffer} data - Content to write
 * @param {object} options - Write options
 * @returns {Promise<void>}
 */
async function writeFile(filePath, data, options = {}) {
  await fsPromises.writeFile(filePath, data, options);
}

/**
 * Create directory (async)
 * Replaces: fse.mkdir()
 * @param {string} dirPath - Path to directory
 * @param {object} options - mkdir options
 * @returns {Promise<void>}
 */
async function mkdir(dirPath, options = {}) {
  await fsPromises.mkdir(dirPath, options);
}

/**
 * Detect Ubuntu codename from /etc/os-release
 * @returns {string|null} Ubuntu codename (e.g., 'noble', 'jammy') or null if not detectable
 */
function detectUbuntuCodename() {
  if (process.platform !== 'linux') {
    return null;
  }

  try {
    const osRelease = fs.readFileSync('/etc/os-release', 'utf8');
    const match = osRelease.match(/^VERSION_CODENAME=(.*)$/m);
    return match ? match[1].trim() : null;
  } catch {
    return null;
  }
}

/**
 * Normalize a ROS 2 node name by removing the leading slash if present.
 *
 * ROS 2 node names may be specified with or without a leading slash depending
 * on the context. This utility ensures consistent representation without the
 * leading slash, which is the standard format for most ROS 2 APIs.
 *
 * @param {string} nodeName - The node name to normalize
 * @returns {string} The normalized node name without leading slash
 *
 * @example
 * normalizeNodeName('my_node')    // 'my_node'
 * normalizeNodeName('/my_node')   // 'my_node'
 * normalizeNodeName('/ns/my_node') // 'ns/my_node'
 */
function normalizeNodeName(nodeName) {
  return nodeName.startsWith('/') ? nodeName.substring(1) : nodeName;
}

/**
 * Check if two numbers are equal within a given tolerance.
 *
 * This function compares two numbers using both relative and absolute tolerance,
 * matching the behavior of the 'is-close' npm package.
 *
 * The comparison uses the formula:
 *   abs(a - b) <= max(rtol * max(abs(a), abs(b)), atol)
 *
 * Implementation checks:
 *   1. Absolute tolerance: abs(a - b) <= atol
 *   2. Relative tolerance: abs(a - b) / max(abs(a), abs(b)) <= rtol
 *
 * @param {number} a - The first number to compare
 * @param {number} b - The second number to compare
 * @param {number} [rtol=1e-9] - The relative tolerance parameter (default: 1e-9)
 * @param {number} [atol=0.0] - The absolute tolerance parameter (default: 0.0)
 * @returns {boolean} True if the numbers are close within the tolerance
 *
 * @example
 * isClose(1.0, 1.0) // true - exact equality
 * isClose(1.0, 1.1, 0.01) // false - relative diff: 0.1/1.1 ≈ 0.091 > 0.01
 * isClose(10, 10.00001, 1e-6) // true - relative diff: 0.00001/10 = 1e-6 <= 1e-6
 * isClose(0, 0.05, 0, 0.1) // true - absolute diff: 0.05 <= 0.1 (atol)
 */
function isClose(a, b, rtol = 1e-9, atol = 0.0) {
  // Handle exact equality
  if (a === b) {
    return true;
  }

  // Handle non-finite numbers
  if (!Number.isFinite(a) || !Number.isFinite(b)) {
    return false;
  }

  const absDiff = Math.abs(a - b);

  // Check absolute tolerance first (optimization)
  if (atol >= absDiff) {
    return true;
  }

  // Check relative tolerance
  const relativeScaler = Math.max(Math.abs(a), Math.abs(b));

  // Handle division by zero when both values are zero or very close to zero
  if (relativeScaler === 0) {
    return true; // Both are zero, already handled by absolute tolerance
  }

  const relativeDiff = absDiff / relativeScaler;

  return rtol >= relativeDiff;
}

/**
 * Compare two semantic version strings.
 *
 * Supports version strings in the format: x.y.z or x.y.z.w
 * where x, y, z, w are integers.
 *
 * @param {string} version1 - First version string (e.g., '1.2.3')
 * @param {string} version2 - Second version string (e.g., '1.2.4')
 * @param {string} operator - Comparison operator: '<', '<=', '>', '>=', '==', '!='
 * @returns {boolean} Result of the comparison
 *
 * @example
 * compareVersions('1.2.3', '1.2.4', '<')   // true
 * compareVersions('2.0.0', '1.9.9', '>')   // true
 * compareVersions('1.2.3', '1.2.3', '==')  // true
 * compareVersions('1.2.3', '1.2.3', '>=')  // true
 */
function compareVersions(version1, version2, operator) {
  // Parse version strings into arrays of integers
  const v1Parts = version1.split('.').map((part) => parseInt(part, 10));
  const v2Parts = version2.split('.').map((part) => parseInt(part, 10));

  // Pad arrays to same length with zeros
  const maxLength = Math.max(v1Parts.length, v2Parts.length);
  while (v1Parts.length < maxLength) v1Parts.push(0);
  while (v2Parts.length < maxLength) v2Parts.push(0);

  // Compare each part
  let cmp = 0;
  for (let i = 0; i < maxLength; i++) {
    if (v1Parts[i] > v2Parts[i]) {
      cmp = 1;
      break;
    } else if (v1Parts[i] < v2Parts[i]) {
      cmp = -1;
      break;
    }
  }

  // Apply operator
  switch (operator) {
    case '<':
      return cmp < 0;
    case '<=':
      return cmp <= 0;
    case '>':
      return cmp > 0;
    case '>=':
      return cmp >= 0;
    case '==':
    case '===':
      return cmp === 0;
    case '!=':
    case '!==':
      return cmp !== 0;
    default:
      throw new ValidationError(`Invalid operator: ${operator}`, {
        code: 'INVALID_OPERATOR',
        argumentName: 'operator',
        providedValue: operator,
        expectedType: "'eq' | 'ne' | 'lt' | 'lte' | 'gt' | 'gte'",
      });
  }
}

module.exports = {
  // General utilities
  detectUbuntuCodename,
  isClose,
  normalizeNodeName,

  // File system utilities (async)
  ensureDir,
  mkdirs: ensureDir, // Alias for fs-extra compatibility
  exists: pathExists, // Renamed to avoid conflict with deprecated fs.exists
  pathExists,
  emptyDir,
  copy,
  remove,
  writeFile,
  mkdir,

  // File system utilities (sync)
  ensureDirSync,
  mkdirSync: ensureDirSync, // Alias for fs-extra compatibility
  removeSync,
  readJsonSync,

  compareVersions,
};
