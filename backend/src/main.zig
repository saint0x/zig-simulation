//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.

const std = @import("std");
const core = @import("core");

pub fn main() !void {
    // Initialize timing system for 1kHz control loop (1ms interval)
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();
    var timing = try core.TimingSystem.init(allocator);

    // Initialize joint manager with default configurations
    var joint_manager = try core.JointManager.init(null, &timing);
    defer joint_manager.deinit();

    // Power up the robot
    joint_manager.reset();

    // Main control loop
    while (true) {
        // Wait for next control cycle
        const current_time: i64 = @intCast(std.time.nanoTimestamp());
        if (!timing.shouldUpdate(current_time)) continue;

        // Update joint controllers
        try joint_manager.update();

        // TODO: Add communication with frontend
        // 1. Receive target positions
        // 2. Send current joint states
        // 3. Handle commands
        // 4. Report errors
    }
}