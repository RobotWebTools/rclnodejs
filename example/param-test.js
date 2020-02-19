
'use strict';


const rclnodejs = require('../index.js');

/*
node example/param-test.js 
  --ros-args 
  -p bool:=true 
  -p int:=101 
  -p dbl:=222.22 
  -p string:="hello" 
  -p bytarr:="[1b, 2b]" 
  -p boolarr:="[true, false, true]" 
  -p intarr:="[101,201]"  
  -p zzz:dblarr:="[5.5, 6.6]" 
  -p zzz:strarr:="[abc, def]"
  -r 
*/
// parameters[] = 
//  {node_name: string
//   parameters[] = {
//     name: string
//     type: uint
//     value: object
//   }

async function main() {
  console.log('Start');

  await rclnodejs.init();
  const parameters = rclnodejs.getParameterOverrides();
  if (!parameters) {
    console.log('No parameters found');
    return;
  }

  console.log('PARAMETERS', parameters);
  console.log('--------------------');
  for (const node of parameters) {
    console.log('   node: ', node.name);
    for (const p of node.parameters) {
      console.log(`  ${p.name}, ${p.type}, ${p.value}`);
    }
  }


  // console.log('hw: ', rclnodejs.helloWorld());
  // console.log('stringarray: ', rclnodejs.stringArrayTest());

  console.log('Complete');
}

main();

