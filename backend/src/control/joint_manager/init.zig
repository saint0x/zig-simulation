const std = @import("std");
const core = @import("core");
const types = @import("types.zig");

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
                .min_position = -std.math.pi,
                .max_position = std.math.pi,
                .max_velocity = 1.0,
                .max_acceleration = 1.0,
            };
        }
    }

    // Initialize joint states
    for (0..core.types.NUM_JOINTS) |i| {
        joint_mgr.joints[i] = .{
            .position = 0,
            .velocity = 0,
            .acceleration = 0,
            .target_position = 0,
            .target_velocity = 0,
        };
    }

    // Initialize timing system
    joint_mgr.timing = timing;

    // Initialize safety monitor
    joint_mgr.safety = try core.safety.SafetyMonitor.init(0.1);

    // Initialize collision detector
    joint_mgr.collision_detector = try core.collision_detection.CollisionDetector.init();

    // Initialize robot state
    joint_mgr.robot_state = .{
        .joints = joint_mgr.joints,
        .is_moving = false,
        .is_error = false,
        .error_code = 0,
    };

    // Initialize error context
    joint_mgr.error_context = .{
        .last_error = null,
        .error_count = 0,
    };

    return joint_mgr;
} 