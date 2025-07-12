const std = @import("std");

// This file makes all public declarations in backend/src/communication/main.zig 
// available when importing the 'communication' module.
pub usingnamespace @import("main.zig");

pub const Communication = @import("main.zig").Communication;
pub const protocol = @import("protocol.zig");

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