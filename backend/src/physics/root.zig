const std = @import("std");
const core = @import("core");

pub const Physics = @import("main.zig").Physics;
pub const simulation = @import("simulation.zig");
pub const types = @import("types.zig");

pub fn init(allocator: std.mem.Allocator) !Physics {
    return Physics.init(allocator);
} 