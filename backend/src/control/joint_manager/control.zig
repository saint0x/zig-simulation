const std = @import("std");
const types = @import("types.zig");
const config = @import("config.zig");

/// Update all joints in the control loop
pub fn update(self: *types.JointManager) void {
    // Skip update if robot is not ready
    if (self.robot_state != .ready and self.robot_state != .moving) {
        return;
    }

    // Update safety monitor
    const dt = 1.0 / @as(f32, @floatFromInt(self.timing.config.control_frequency));
    self.safety.update(&self.states, dt);

    // Check safety status
    switch (self.safety.getStatus()) {
        .fault => {
            self.robot_state = .fault;
            return;
        },
        .emergency_stop => {
            self.robot_state = .emergency_stop;
            return;
        },
        else => {},
    }

    // Update each joint
    for (0..types.NUM_JOINTS) |i| {
        // Update PID controller and get control signal
        // This will be used when we implement the hardware interface
        _ = self.controllers[i].update();

        // Update joint state
        self.states[i] = self.controllers[i].state;
    }
}

/// Set target positions for all joints
pub fn setTargets(self: *types.JointManager, target_angles: [types.NUM_JOINTS]f32, target_velocities: [types.NUM_JOINTS]f32) void {
    // Check if robot is ready
    if (self.robot_state != .ready and self.robot_state != .moving) {
        return;
    }

    // Check if target angles are within limits
    if (!self.safety.checkJointAngles(&target_angles)) {
        self.robot_state = .fault;
        return;
    }

    // Update each joint's target
    for (0..types.NUM_JOINTS) |i| {
        // Clamp target angle to joint limits
        const clamped_angle = std.math.clamp(
            target_angles[i],
            self.configs[i].min_angle,
            self.configs[i].max_angle
        );
        
        // Clamp target velocity to joint limits
        const clamped_velocity = std.math.clamp(
            target_velocities[i],
            -self.configs[i].max_velocity,
            self.configs[i].max_velocity
        );

        self.controllers[i].setTarget(clamped_angle, clamped_velocity);
    }

    // Update robot state to moving
    self.robot_state = .moving;
} 