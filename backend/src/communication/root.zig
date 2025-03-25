const std = @import("std");
const core = @import("core");

pub const Communication = @import("main.zig").Communication;

pub fn init(allocator: std.mem.Allocator) !Communication {
    return Communication.init(allocator);
} 