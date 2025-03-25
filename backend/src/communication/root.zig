const std = @import("std");
const core = @import("core");

pub const Communication = @import("main.zig").Communication;

pub fn init(allocator: std.mem.Allocator) !*Communication {
    const comm = try allocator.create(Communication);
    comm.* = try Communication.init(allocator);
    return comm;
}

pub fn deinit(comm: *Communication) void {
    const allocator = comm.allocator;
    comm.deinit();
    allocator.destroy(comm);
}

pub fn start(comm: *Communication) !void {
    try comm.start();
} 