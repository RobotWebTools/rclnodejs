const childprocess = require('child_process');
const os = require('os');

process.env.JOBS = os.cpus().length;
childprocess.execSync('node-gyp rebuild', {
  stdio: 'inherit',
});
