//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.

const std = @import("std");
const core = @import("core");
const joints = @import("joints");
const safety = @import("safety");
const kinematics = @import("kinematics");

pub fn main() !void {
    // Initialize timing system for 1kHz control loop (1ms interval)
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();
    var timing = try core.TimingSystem.init(allocator);

    // Initialize link dimensions for forward kinematics
    const link_dimensions = [_]core.types.LinkDimensions{
        .{
            .id = .base,
            .width = 0.2,
            .height = 0.3,
            .length = 0.2,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0 },
        },
        .{
            .id = .shoulder,
            .width = 0.15,
            .height = 0.4,
            .length = 0.4,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.3 },
        },
        .{
            .id = .upper_arm,
            .width = 0.12,
            .height = 0.35,
            .length = 0.4,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.4 },
        },
        .{
            .id = .elbow,
            .width = 0.1,
            .height = 0.3,
            .length = 0.35,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.35 },
        },
        .{
            .id = .forearm,
            .width = 0.08,
            .height = 0.25,
            .length = 0.3,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.3 },
        },
        .{
            .id = .wrist_rotation,
            .width = 0.07,
            .height = 0.2,
            .length = 0.2,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.2 },
        },
        .{
            .id = .wrist_bend,
            .width = 0.06,
            .height = 0.15,
            .length = 0.15,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.15 },
        },
        .{
            .id = .tool_rotation,
            .width = 0.05,
            .height = 0.1,
            .length = 0.1,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.1 },
        },
        .{
            .id = .gripper,
            .width = 0.05,
            .height = 0.1,
            .length = 0.1,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0.1 },
        },
    };

    // Initialize kinematics
    var fk = kinematics.ForwardKinematics.init(link_dimensions);

    // Initialize collision detection configuration
    const collision_config = core.types.CollisionConfig{
        .min_link_distance = 0.01, // 1cm minimum distance between links
        .min_environment_distance = 0.05, // 5cm minimum distance from environment
        .enable_continuous_detection = true,
        .check_frequency = 100, // Check for collisions 100 times per second
        .link_dimensions = link_dimensions,
    };

    // Initialize collision detection
    const collision_detector = try kinematics.collision_detection.CollisionDetection.init(
        collision_config,
        &fk
    );

    // Initialize joint limits
    const joint_limits = [_]safety.limits.JointLimits{
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
    };

    // Initialize safety monitor
    var safety_monitor = safety.SafetyMonitor.init(
        &joint_limits,
        collision_detector,
        10.0 // Emergency stop threshold in degrees/second
    );

    // Initialize joint manager with default configurations
    var joint_manager = try joints.JointManager.init(
        allocator,
        &timing,
        &safety_monitor,
        collision_detector,
        &fk
    );
    defer joint_manager.deinit();

    // Power up the robot
    joint_manager.reset();

    // Main control loop
    while (true) {
        // Wait for next control cycle
        const current_time: i64 = @intCast(std.time.nanoTimestamp());
        if (!timing.shouldUpdate(current_time)) continue;

        // Update joint controllers
        try joint_manager.updateStates();

        // TODO: Add communication with frontend
        // 1. Receive target positions
        // 2. Send current joint states
        // 3. Handle commands
        // 4. Report errors
    }
}