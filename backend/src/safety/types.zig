const std = @import("std");
const core_types = @import("core").types;

pub const SafetyConfig = struct {
    joint_limits: [core_types.NUM_JOINTS]JointLimit,
    collision_config: core_types.CollisionConfig,
    emergency_stop_threshold: f32,
};

pub const JointLimit = struct {
    min_angle: f32,
    max_angle: f32,
    max_velocity: f32,
    max_acceleration: f32,
}; 