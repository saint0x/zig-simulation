const std = @import("std");
const core = @import("core");

pub const NUM_JOINTS = core.types.NUM_JOINTS;

pub const ControlConfig = struct {
    pid_config: PIDConfig,
    joint_config: core.types.JointConfig,
    motion_config: MotionConfig,
};

pub const PIDConfig = struct {
    kp: f32,
    ki: f32,
    kd: f32,
    min_output: f32,
    max_output: f32,
};

pub const MotionConfig = struct {
    max_velocity: f32,
    max_acceleration: f32,
    max_jerk: f32,
    min_path_radius: f32,
}; 