const std = @import("std");

pub fn getTimestamp() i64 {
    return std.time.timestamp();
}

pub fn sleep(milliseconds: u64) void {
    std.time.sleep(milliseconds * std.time.ns_per_ms);
}

pub fn getMonotonicTime() u64 {
    return std.time.milliTimestamp();
} 