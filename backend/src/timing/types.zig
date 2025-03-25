const std = @import("std");

pub const TimingSystem = struct {
    allocator: std.mem.Allocator,
    last_update: i64,
    update_interval: i64,
    
    pub fn init(allocator: std.mem.Allocator) !TimingSystem {
        return TimingSystem{
            .allocator = allocator,
            .last_update = 0,
            .update_interval = 10_000_000, // 10ms in nanoseconds
        };
    }
    
    pub fn deinit(self: *TimingSystem) void {
        // Cleanup resources if needed
        _ = self;
    }

    pub fn shouldUpdate(self: *TimingSystem, current_time: i64) bool {
        if (current_time - self.last_update >= self.update_interval) {
            self.last_update = current_time;
            return true;
        }
        return false;
    }
};

pub const TimingConfig = struct {
    update_interval: i64,
    watchdog_timeout: i64,
};

pub const TimeUnit = enum {
    Milliseconds,
    Microseconds,
    Nanoseconds,
};

pub const TimerConfig = struct {
    duration: i64,
    unit: TimeUnit,
}; 