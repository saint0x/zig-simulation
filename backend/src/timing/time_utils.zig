const std = @import("std");

pub const TimeConversion = struct {
    pub fn millisToNanos(millis: i64) i64 {
        return millis * std.time.ns_per_ms;
    }

    pub fn nanosToMillis(nanos: i64) i64 {
        return nanos / std.time.ns_per_ms;
    }

    pub fn secondsToNanos(seconds: i64) i64 {
        return seconds * std.time.ns_per_s;
    }

    pub fn nanosToSeconds(nanos: i64) i64 {
        return nanos / std.time.ns_per_s;
    }
}; 