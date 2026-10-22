// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
// Licensed under the Apache License, Version 2.0.

export const TOPIC = '/joint_states';
export const MESSAGE_TYPE = 'sensor_msgs/msg/JointState';
export const JOINT_NAMES = ['joint1', 'joint2'];
export const LIMITS = [Math.PI, (3 * Math.PI) / 4];
export const STEP = (5 * Math.PI) / 180;

export class JointController {
  positions = [0, 0];
  animated = false;
  elapsed = 0;

  move(index, delta) {
    if (![0, 1].includes(index) || !Number.isFinite(delta)) {
      throw new TypeError('Expected a joint index (0 or 1) and a finite angle');
    }
    this.animated = false;
    this.positions[index] = Math.max(
      -LIMITS[index],
      Math.min(LIMITS[index], this.positions[index] + delta)
    );
  }

  reset() {
    this.positions = [0, 0];
    this.animated = false;
    this.elapsed = 0;
  }

  toggleAnimation() {
    this.animated = !this.animated;
    if (this.animated) this.elapsed = 0;
  }

  tick(seconds) {
    if (!Number.isFinite(seconds) || seconds < 0) {
      throw new TypeError('Elapsed time must be finite and nonnegative');
    }
    if (this.animated) {
      this.elapsed += seconds;
      this.positions[0] = (Math.sin(this.elapsed * 0.4) * Math.PI) / 3;
      this.positions[1] = (Math.sin(this.elapsed * 0.6) * Math.PI) / 4;
    }
  }

  toMessage(milliseconds = Date.now()) {
    if (!Number.isFinite(milliseconds) || milliseconds < 0) {
      throw new TypeError('Timestamp must be finite and nonnegative');
    }
    return {
      header: {
        stamp: {
          sec: Math.floor(milliseconds / 1000),
          nanosec: Math.floor((milliseconds % 1000) * 1e6),
        },
        frame_id: 'base_link',
      },
      name: [...JOINT_NAMES],
      position: [...this.positions],
      // This demo publishes positions, not measured velocities or efforts.
      velocity: [],
      effort: [],
    };
  }
}

export function readJointPositions(message) {
  if (
    !Array.isArray(message?.name) ||
    !Array.isArray(message?.position) ||
    message.name.length !== message.position.length
  ) {
    throw new TypeError(
      'JointState name and position arrays must have equal lengths'
    );
  }
  return JOINT_NAMES.map((name) => {
    const index = message.name.indexOf(name);
    if (
      index === -1 ||
      message.name.lastIndexOf(name) !== index ||
      !Number.isFinite(message.position[index])
    ) {
      throw new TypeError(
        `JointState must include one finite position for ${name}`
      );
    }
    return message.position[index];
  });
}
