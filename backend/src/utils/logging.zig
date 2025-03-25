const std = @import("std");

pub const LOG_LEVEL_DEBUG = "DEBUG";
pub const LOG_LEVEL_INFO = "INFO";
pub const LOG_LEVEL_WARNING = "WARNING";
pub const LOG_LEVEL_ERROR = "ERROR";

pub fn log(level: []const u8, comptime format: []const u8, args: anytype) void {
    const stderr = std.io.getStdErr().writer();
    const timestamp = std.time.timestamp();
    
    stderr.print("[{d}][{s}] ", .{
        timestamp,
        level,
    }) catch return;
    
    stderr.print(format ++ "\n", args) catch return;
} 