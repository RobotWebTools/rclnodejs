'use strict';

// helper module to trigger gc on exit, useful to remove false positives when running with ASAN/LSAN

// requires `--expose-gc` flag

// use this with `mocha -r test/gc-on-exit.js`

// to remove leaks not from rclnodejs, run with env LSAN_OPTIONS=suppressions=suppr.txt

process.on('exit', global.gc);
