const std = @import("std");
const types = @import("types.zig");
const config = @import("config.zig");

/// Update current joint states
pub fn updateStates(self: *types.JointManager, current_angles: [types.NUM_JOINTS]f32, current_velocities: [types.NUM_JOINTS]f32) void {
    // Check if current angles are within limits
    if (!self.safety.checkJointAngles(&current_angles)) {
        self.robot_state = .fault;
        return;
    }

    // Update each joint's state
    for (0..types.NUM_JOINTS) |i| {
        self.controllers[i].updateState(current_angles[i], current_velocities[i]);
    }
}

/// Get the current state of a specific joint
pub fn getJointState(self: *const types.JointManager, joint_id: types.JointId) types.JointState {
    return self.states[@intFromEnum(joint_id)];
}

/// Get the current robot state
pub fn getRobotState(self: *const types.JointManager) types.RobotState {
    return self.robot_state;
}

/// Check if all joints have reached their target positions
pub fn hasReachedTarget(self: *const types.JointManager) bool {
    for (0..types.NUM_JOINTS) |i| {
        const angle_error = @abs(
            self.states[i].target_angle - self.states[i].current_angle
        );
        if (angle_error > 0.1) { // 0.1 degree tolerance
            return false;
        }
    }
    return true;
} 