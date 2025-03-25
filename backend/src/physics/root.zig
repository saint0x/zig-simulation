const std = @import("std");
const core = @import("core");

pub const Physics = @import("main.zig").Physics;

pub fn init(allocator: std.mem.Allocator) !Physics {
    return Physics.init(allocator);
} 