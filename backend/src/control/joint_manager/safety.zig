const std = @import("std");
const types = @import("types.zig");
const config = @import("config.zig");

/// Reset all joints to their initial state
pub fn reset(self: *types.JointManager) void {
    for (0..types.NUM_JOINTS) |i| {
        self.controllers[i].reset();
        self.states[i] = self.controllers[i].state;
    }
    self.safety.reset();
    self.robot_state = .ready;
} 