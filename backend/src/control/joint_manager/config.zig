const types = @import("types.zig");

/// Default joint configurations for the KUKA arm
pub const default_joint_configs = [types.NUM_JOINTS]types.JointConfig{
    // Base rotation
    .{
        .id = .base_rotation,
        .min_angle = -180,
        .max_angle = 180,
        .max_velocity = 120,
        .max_acceleration = 180,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 2.0,
            .ki = 0.1,
            .kd = 0.5,
            .i_max = 10.0,
        },
    },
    // Shoulder rotation
    .{
        .id = .shoulder_rotation,
        .min_angle = -90,
        .max_angle = 90,
        .max_velocity = 90,
        .max_acceleration = 135,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 2.5,
            .ki = 0.15,
            .kd = 0.6,
            .i_max = 8.0,
        },
    },
    // Elbow rotation
    .{
        .id = .elbow_rotation,
        .min_angle = -120,
        .max_angle = 120,
        .max_velocity = 150,
        .max_acceleration = 225,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 2.0,
            .ki = 0.1,
            .kd = 0.4,
            .i_max = 12.0,
        },
    },
    // Wrist bend
    .{
        .id = .wrist_bend,
        .min_angle = -135,
        .max_angle = 135,
        .max_velocity = 180,
        .max_acceleration = 270,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 1.8,
            .ki = 0.08,
            .kd = 0.35,
            .i_max = 15.0,
        },
    },
    // Wrist rotation
    .{
        .id = .wrist_rotation,
        .min_angle = -180,
        .max_angle = 180,
        .max_velocity = 200,
        .max_acceleration = 300,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 1.5,
            .ki = 0.05,
            .kd = 0.3,
            .i_max = 20.0,
        },
    },
    // Tool rotation
    .{
        .id = .tool_rotation,
        .min_angle = -360,
        .max_angle = 360,
        .max_velocity = 250,
        .max_acceleration = 375,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 1.2,
            .ki = 0.03,
            .kd = 0.25,
            .i_max = 25.0,
        },
    },
    // Gripper
    .{
        .id = .gripper,
        .min_angle = -45,
        .max_angle = 45,
        .max_velocity = 90,
        .max_acceleration = 135,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 3.0,
            .ki = 0.2,
            .kd = 0.7,
            .i_max = 5.0,
        },
    },
}; 