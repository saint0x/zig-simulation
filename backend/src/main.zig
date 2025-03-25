//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.

const std = @import("std");
const core = @import("core");

pub fn main() !void {
    // Initialize timing system for 1kHz control loop
    var timing = core.TimingSystem.init(.{
        .control_frequency = 1000,
        .max_jitter = 100,
    });

    // Initialize joint manager with default configurations
    var joint_manager = core.JointManager.init(null, &timing);

    // Power up the robot
    joint_manager.reset();

    // Main control loop
    while (true) {
        // Wait for next control cycle
        _ = timing.waitForNextTick();

        // Update joint controllers
        joint_manager.update();

        // TODO: Add communication with frontend
        // 1. Receive target positions
        // 2. Send current joint states
        // 3. Handle commands
        // 4. Report errors
    }
}