// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Public entry for `rclnodejs/web`. Re-exports the named SDK surface
// from client.js so consumers can:
//
//   import { connect, RosClient } from 'rclnodejs/web';

export { connect, RosClient } from './client.js';
