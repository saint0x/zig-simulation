const std = @import("std");
const utils = @import("utils");
const types = @import("types.zig");

pub const TimingSystem = types.TimingSystem;

pub fn init(allocator: std.mem.Allocator) !TimingSystem {
    utils.log(utils.LOG_LEVEL_INFO, "Initializing timing system", .{});
    return TimingSystem.init(allocator);
} 