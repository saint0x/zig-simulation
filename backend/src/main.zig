//! KUKA Robot Arm Control System - Main Entry Point
//!
//! This is the production entry point for the KUKA robot arm simulation backend.
//! It initializes all subsystems and runs the 1kHz real-time control loop.

const std = @import("std");
const core = @import("core");
const joints = @import("joints");
const safety = @import("safety");
const kinematics = @import("kinematics");
const communication = @import("communication");
const posix = std.posix;

// Global shutdown flag for signal handling (atomic for thread safety)
var g_shutdown_requested = std.atomic.Value(bool).init(false);

/// Signal handler for graceful shutdown (SIGINT, SIGTERM)
fn signalHandler(sig: i32) callconv(.C) void {
    _ = sig;
    g_shutdown_requested.store(true, .seq_cst);
}

/// Setup signal handlers for graceful shutdown
fn setupSignalHandlers() !void {
    const sigaction_handler = posix.Sigaction{
        .handler = .{ .handler = signalHandler },
        .mask = posix.empty_sigset,
        .flags = 0,
    };

    // Handle SIGINT (Ctrl+C)
    try posix.sigaction(posix.SIG.INT, &sigaction_handler, null);
    // Handle SIGTERM (kill command)
    try posix.sigaction(posix.SIG.TERM, &sigaction_handler, null);
}

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

    // Initialize collision detection configuration (temporarily disabled for testing)
    const collision_config = core.types.CollisionConfig{
        .min_link_distance = 0.01, // 1cm minimum distance between links
        .min_environment_distance = 0.05, // 5cm minimum distance from environment
        .enable_continuous_detection = false, // Temporarily disabled
        .check_frequency = 100, // Check for collisions 100 times per second
        .link_dimensions = link_dimensions,
    };

    // Initialize collision detection
    const collision_detector = try kinematics.collision_detection.CollisionDetection.init(
        collision_config,
        &fk
    );

    // Initialize joint limits (7 joints total) with realistic KUKA KR6 limits
    const joint_limits = [_]safety.types.JointLimit{
        .{ .min_angle = -180, .max_angle = 180, .max_velocity = 160, .max_acceleration = 300 }, // base_rotation
        .{ .min_angle = -180, .max_angle = 180, .max_velocity = 135, .max_acceleration = 250 },  // shoulder_rotation  
        .{ .min_angle = -180, .max_angle = 180, .max_velocity = 190, .max_acceleration = 350 }, // elbow_rotation
        .{ .min_angle = -180, .max_angle = 180, .max_velocity = 210, .max_acceleration = 400 }, // wrist_bend
        .{ .min_angle = -180, .max_angle = 180, .max_velocity = 230, .max_acceleration = 450 }, // wrist_rotation
        .{ .min_angle = -180, .max_angle = 180, .max_velocity = 290, .max_acceleration = 500 }, // tool_rotation
        .{ .min_angle = -180, .max_angle = 180, .max_velocity = 130, .max_acceleration = 200 },  // gripper
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

    // Initialize and start the WebSocket server
    const web_socket_port: u16 = 9001;
    const comm_server = try communication.WebSocketServer.init(allocator, web_socket_port, joint_manager);
    
    // Spawn a thread for the communication server to listen for connections
    var server_thread = try std.Thread.spawn(.{}, struct {
        fn run(server: *communication.WebSocketServer) !void {
            try server.start();
        }
    }.run, .{comm_server});
    // Don't detach - we need to join on shutdown

    std.log.info("WebSocket server started on port {d}", .{web_socket_port});

    // Setup signal handlers for graceful shutdown
    setupSignalHandlers() catch |err| {
        std.log.warn("Failed to setup signal handlers: {} - shutdown may not be graceful", .{err});
    };

    // Power up the robot
    joint_manager.reset();

    var iteration_count: u64 = 0;

    std.log.info("Starting main control loop (press Ctrl+C to shutdown gracefully)", .{});

    // Main control loop - production mode with graceful shutdown support
    while (!g_shutdown_requested.load(.seq_cst)) {
        // Wait for next control cycle
        const current_time: i64 = @intCast(std.time.nanoTimestamp());
        if (!timing.shouldUpdate(current_time)) continue;

        // Update joint controllers with error handling
        joint_manager.updateStates() catch |err| {
            std.log.err("Joint update error: {}", .{err});
            if (err == error.SafetyLimitExceeded) {
                std.log.warn("Safety limits exceeded - resetting to safe state", .{});
                joint_manager.reset();
                continue;
            }
            break; // Exit on other errors
        };

        iteration_count += 1;

        // Periodic status logging (every 10 seconds at 1kHz = 10000 iterations)
        if (iteration_count % 10000 == 0) {
            std.log.info("Control loop running: {} iterations completed", .{iteration_count});
        }
    }

    std.log.info("Shutdown requested. Completing {} iterations...", .{iteration_count});
    std.log.info("Stopping WebSocket server...", .{});

    // Proper shutdown sequence: stop server first, then wait for thread
    comm_server.stop();
    server_thread.join();

    std.log.info("Shutdown complete.", .{});
}