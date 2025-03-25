const std = @import("std");
const core = @import("core");
const types = @import("types.zig");

pub const timing = @import("timing.zig");

// Re-export commonly used types
pub const TimingConfig = core.types.TimingConfig;
pub const TimeUnit = core.types.TimeUnit;
pub const TimerConfig = core.types.TimerConfig;
pub const TimingSystem = struct {
    last_update: i64,
    update_interval: i64,
    
    pub fn init(update_interval: i64) TimingSystem {
        return TimingSystem{
            .last_update = 0,
            .update_interval = update_interval,
        };
    }

    pub fn shouldUpdate(self: *TimingSystem, current_time: i64) bool {
        const elapsed = current_time - self.last_update;
        if (elapsed >= self.update_interval) {
            self.last_update = current_time;
            return true;
        }
        return false;
    }

    pub fn getElapsedTime(self: *const TimingSystem, current_time: i64) i64 {
        return current_time - self.last_update;
    }
};

pub const Timer = struct {
    start_time: i64,
    duration: i64,

    pub fn init(duration: i64) Timer {
        return Timer{
            .start_time = 0,
            .duration = duration,
        };
    }

    pub fn start(self: *Timer, current_time: i64) void {
        self.start_time = current_time;
    }

    pub fn hasElapsed(self: *const Timer, current_time: i64) bool {
        return (current_time - self.start_time) >= self.duration;
    }
}; 