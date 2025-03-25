const std = @import("std");
const types = @import("types.zig");
const utils = @import("utils");

pub const CoreSystem = struct {
    allocator: std.mem.Allocator,
    
    pub fn init(allocator: std.mem.Allocator) !CoreSystem {
        return CoreSystem{
            .allocator = allocator,
        };
    }
    
    pub fn deinit(self: *CoreSystem) void {
        // Cleanup resources if needed
        _ = self;
    }
    
    pub fn run(self: *CoreSystem) !void {
        utils.log(utils.LOG_LEVEL_INFO, "Core system initialized and running", .{});
        _ = self;
    }
};

pub fn init(allocator: std.mem.Allocator) !CoreSystem {
    return CoreSystem.init(allocator);
} 