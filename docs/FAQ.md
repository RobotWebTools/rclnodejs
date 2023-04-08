# FAQ and Known Issues

Have a question or issue that should be included in this document? Consider opening an issue or discussion with the [rclnodejs project team](https://github.com/RobotWebTools/rclnodejs). 

## Undefined or missing JavaScript message file
The most common reasons for this error are:
* Misspelling of a message name in your code
* The ROS package containing the target message is not installed in your ROS environment 
* The ROS package containing the target message was installed after the `rclnodejs` package was installed in your ROS2-nodejs environment.

#### Verify the ROS package is installed
The first step is to verify that your ROS environment includes the package containing the target message. Using the `ros2` cli list all of the packages in your ROS environment and inspect the output for the package.
```
ros2 pkg list
```
If the package containing your target message is not listed then install it.

Next, inspect the generated JavaScript message files by viewing the `./node_modules/rclnodejs/generated/` folder of your project for your target message. If you are unable to locate the message file then use the `generate-messages` script:
```
<your_project>/node_modules/.bin/generate-messages
```


## Maximum call stack size exceeded error when running in Jest

When running tests in Jest, you may see an error like this:

```
RangeError: Maximum call-stack size exceeded

      at debug (../node_modules/ref/node_modules/debug/src/debug.js:1:1)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:746:3)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/
      ...
```

This is caused by a bug in `ref` which happens when you `require` it multiple times. There is a fix available for `ref` but it's no longer being maintained and the author has not published it.

If it is required to use Jest, a solution would be to fork `ref` and use npm shrinkwrap to installed a patched version.

## Running with ASAN

#### Linux

To run with google's AddressSanitizer tool, build with `-fsanitize=address` flag,

```sh
CXXFLAGS=-fsanitize=address node-gyp build --debug
```

ASAN needs to be loaded at the start of the process, since rclnodejs is a dynamically loaded library, it will not do so by default. To workaround this, run node with `LD_PRELOAD` to force ASAN to be loaded.

```sh
LD_PRELOAD=$(g++ -print-file-name=libasan.so) node node_modules/.bin/mocha test/test-publisher.js
```

Due to v8's garbage collector, there may be false positives in the leak test, to remove them as much as possible, there is a simple helper script to run gc on exit. To use it, the `--expose-gc` flag needs to be set in node, then run mocha with `-r test/gc-on-exit.js` e.g.

```sh
LD_PRELOAD=$(g++ -print-file-name=libasan.so) node --expose-gc node_modules/.bin/mocha -r test/gc-on-exit.js test/test-publisher.js
```

**Note**: Tests that forks the current process like `test-array.js` will not run gc when they exit. They may report many false positive leaks.

ASAN may report leaks in ref-napi and other modules, there is a suppression file you can use to hide them

```sh
LSAN_OPTIONS=suppressions=suppr.txt node --expose-gc node_modules/.bin/mocha -r test/gc-on-exit.js test/test-publisher.js
```
## FastDDS requirements for multiple node processes on the same machine running under different users
When running multiple ROS 2 node processes configured to use FastDDS ROS middleware (rmw), you can experience a communications failure when the ROS nodes are run under different Linux user accounts. The issue has to do with FastDDS use of shared memory and Linux restrictions on accessing such memory. The workaround is to disable the use of FastDDS shared memory via a configuration file as outlined [here](https://github.com/eProsima/Fast-DDS/issues/1750).