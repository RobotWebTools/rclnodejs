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

const generateJSStructFromIDL = require('./idl_generator.js');
const packages = require('./packages.js');
const path = require('path');

const generatedRoot = path.join(__dirname, '../generated/');
const useIDL = !!process.argv.find((arg) => arg === '--idl');

// Get target path from environment variable instead of workerData
const targetPath = process.env.WORKER_TARGET_PATH;

async function generateInPath(targetPath) {
  let pkgsInfo = null;
  if (!useIDL) {
    pkgsInfo = Array.from(
      (await packages.findPackagesInDirectory(targetPath)).values()
    );
  } else {
    // Direct IDL parsing: pass .idl files to the generator which uses
    // rosidl_parser to parse them directly (no .msg/.srv/.action conversion).
    const idlPkgs = await packages.findPackagesInDirectory(targetPath, useIDL);
    pkgsInfo = Array.from(idlPkgs.values());
  }

  await Promise.all(
    pkgsInfo.map((pkgInfo) => generateJSStructFromIDL(pkgInfo, generatedRoot))
  );
}

async function main() {
  try {
    await generateInPath(targetPath);
    process.exit(0);
  } catch (error) {
    console.error('Worker generation failed:', error.message);
    process.exit(1);
  }
}

main();
