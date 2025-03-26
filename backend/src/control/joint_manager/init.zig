const std = @import("std");
const core = @import("core");
const types = @import("types.zig");
const kinematics = @import("kinematics");
const safety = @import("safety");

/// Initialize a new joint manager with default or custom configurations
pub fn init(configs: ?[core.types.NUM_JOINTS]core.types.JointConfig, timing: *core.TimingSystem) !*types.JointManager {
    var joint_mgr = try std.heap.page_allocator.create(types.JointManager);
    
    // Initialize joint configurations
    if (configs) |joint_configs| {
        joint_mgr.configs = joint_configs;
    } else {
        // Default configurations
        for (0..core.types.NUM_JOINTS) |i| {
            joint_mgr.configs[i] = .{
                .id = @enumFromInt(i),
                .min_angle = -std.math.pi,
                .max_angle = std.math.pi,
                .max_velocity = 1.0,
                .max_acceleration = 1.0,
                .dt = 0.01,
                .pid_gains = .{
                    .kp = 1.0,
                    .ki = 0.1,
                    .kd = 0.01,
                    .i_max = 1.0,
                },
            };
        }
    }

    // Initialize joint states
    for (0..core.types.NUM_JOINTS) |i| {
        joint_mgr.joints[i] = .{
            .current_angle = 0,
            .current_velocity = 0,
            .target_angle = 0,
            .target_velocity = 0,
            .current_torque = 0,
            .temperature = 25.0,  // Room temperature
            .current = 0,
            .integral_term = 0,
            .last_error = 0,
        };
    }

    // Initialize timing system
    joint_mgr.timing_system = timing;

    // Initialize collision detector
    joint_mgr.collision_detector = try kinematics.collision_detection.CollisionDetection.init(
        .{
            .min_link_distance = 0.1,
            .min_environment_distance = 0.2,
            .enable_continuous_detection = true,
            .check_frequency = 100,
            .link_dimensions = undefined, // TODO: Set actual link dimensions
        },
        undefined, // TODO: Set forward kinematics
    );

    // Initialize safety monitor
    var joint_limits: [core.types.NUM_JOINTS]safety.JointLimits = undefined;
    for (0..core.types.NUM_JOINTS) |i| {
        joint_limits[i] = safety.JointLimits.init(
            joint_mgr.configs[i].min_angle,
            joint_mgr.configs[i].max_angle,
        );
    }
    joint_mgr.safety = safety.SafetyMonitor.init(
        &joint_limits,
        joint_mgr.collision_detector,
        0.1, // Emergency stop threshold
    );

    // Initialize robot state
    joint_mgr.robot_state = .powered_off;

    // Initialize error handler
    joint_mgr.error_handler = core.error_handler.ErrorHandler.init(std.heap.page_allocator);

    return joint_mgr;
} 