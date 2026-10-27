# Build rclnodejs

## Requirements

- **ROS 2:** Humble, Jazzy, Kilted, Lyrical, or Rolling. Follow the [Linux](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debians.html) or [Windows](https://docs.ros.org/en/lyrical/Installation/Windows-Install-Binary.html) installation guide for your distro. Use `--merge-install` when building ROS from source.
- **Node.js:** runtime minimum **20.20.2**. For source development, use **Node 24.11+ in the 24.x line**; the bundler also supports **22.18+ in the 22.x line** and **26+**. Published npm packages already contain the bundles.
- **npm:** use **11.19.0** for source development to avoid npm 10 dependency-resolution failures. This is not a requirement for installing the published package.
- **Native build tools:** Python 3 (`python3` on Linux/macOS, `py -3` on Windows) and the [node-gyp prerequisites](https://github.com/nodejs/node-gyp#installation). Python 2 is unsupported. On Windows, use a Visual Studio Native Tools command prompt.

### C++ Toolchain Requirements

- **Linux:** C++17 for Humble/Jazzy/Kilted; C++20 for Lyrical/Rolling.
- **Windows:** MSBuild inherits the Node/Electron headers' compiler settings; the ROS-specific `cflags_cc` entries do not configure MSVC. Node 20 headers select C++17 even for Lyrical/Rolling, so verify the generated flags before using that combination.

Electron native builds use their own headers and memory-cage handling; the npm runtime's exact Node patch-version minimum is not enforced against those headers.

Native compatibility branches for ROS releases older than Humble have been
removed. Humble and newer API differences remain handled by the build and
runtime code; recognizing a historical distro name does not imply build support.

## Build from Source

```bash
git clone https://github.com/RobotWebTools/rclnodejs.git
cd rclnodejs
npm install --global npm@11.19.0
```

Load your ROS environment in the same terminal. For a Linux binary installation:

```bash
source /opt/ros/jazzy/setup.bash
```

Replace `jazzy` with your distro. For a source installation, source `<install-prefix>/local_setup.bash`. On Windows:

```bat
call C:\path\to\ros2\local_setup.bat
```

Then install and build:

```bash
npm install
npm run build:dist
```

`npm install` uses a prebuilt native addon or builds one, then generates ROS interfaces. `build:dist` creates the ESM/CommonJS package entry points in `dist/`. Source examples can run without this bundling step.

To refresh stale interfaces, including caches reused after skipping install hooks, run `npm run generate-messages` with ROS sourced. The command prepares the native addon before generating messages and TypeScript declarations.

## Build and Run with Docker

The [Dockerfile](../Dockerfile) provides a source-development environment with **Rolling, Node 24 LTS, and npm 11.19.0** by default:

```bash
docker build -t rclnodejs .
docker run --rm -it rclnodejs
```

Override `ROS_DISTRO`, `NODE_MAJOR_VER`, or `BRANCH` with `--build-arg`. The image clones the selected GitHub branch (default: `develop`), **not the local checkout**, and its entrypoint loads ROS automatically.

Source examples run directly; execute `npm run build:dist` inside the container before using bundled package entry points. The image does not include the extra Electron/Xvfb dependencies for the full test suite.

## Tests

With ROS sourced and the native addon built:

| Command                | Purpose                                                                                              |
| ---------------------- | ---------------------------------------------------------------------------------------------------- |
| `npm test`             | Mocha, TypeScript, and Electron tests.                                                               |
| `npm run test:package` | Build bundles and validate an installed tarball using the existing addon; install hooks are skipped. |
| `npm run lint`         | JavaScript and native-code linting.                                                                  |

On Linux, Electron tests need GUI libraries and, when headless, `xvfb-run`; see the [CI dependency setup](../.github/workflows/linux-x64-build-and-test.yml). Packaging tests require a bundler-supported Node version. The pack-based CLI tests are skipped on Node 20; ordinary runtime tests still run.

Linux CI uses official ROS base images for Humble, Jazzy, Kilted, Lyrical, and Rolling on x64 and ARM64. Both Linux build workflows share [test dependency setup](../.github/actions/setup-ros2-apt/action.yml) and explicitly load ROS before building. Rolling keeps Node 26 coverage and the ARM64 IDL suite; its image is not guaranteed to match the latest upstream source.

Windows CI covers Humble, Jazzy, Kilted, and Lyrical, with full tests on Lyrical. Windows Rolling is not covered by CI.

### Nightly compatibility checks

The [Nightly Compatibility workflow](../.github/workflows/nightly-compatibility.yml) builds from source and runs core ROS, TypeScript, and package tests on Ubuntu 26.04 x64 with ROS Rolling nightly, Node 27 nightly, and bundled npm.

- **Scheduled:** daily at 00:00 Asia/Shanghai (16:00 UTC) on the default branch; execution may be delayed.
- **Manual:** use **Actions > Nightly Compatibility > Run workflow**. The `node-version` input defaults to `27-nightly`; use `26.X` for comparison.

For pre-merge testing, enable Actions in your fork and place the workflow on its default branch.

### Coverage

```bash
npm run test:coverage
```

Reports: `coverage/lcov.info` and `coverage/lcov-report/index.html`. Local generation does not upload coverage; CI uploads only from the Lyrical/Node 24 lane.

### C++ Formatting

Run `npm run format`. It uses the npm-installed `clang-format-node` (`^4.0.3`; tested: 4.0.3 / LLVM 23.1.3) and [.clang-format](../.clang-format). No separate formatter installation is needed. The C++20 formatting mode does not set the compiler standard.

## Packaging

```bash
npm pack
```

The `prepack` hook builds the JavaScript bundles automatically.

For a release tarball with prebuilt addons, run the [NPM Publish workflow](../.github/workflows/npm-publish.yml) with `dry_run: true`, then download the `rclnodejs-npm-tarball` artifact and extract its `.tgz`.

Dry runs build and package only: they neither publish nor validate registry version availability, publishing credentials, or provenance. Tag-triggered runs and manual runs with `dry_run: false` publish to npm.
