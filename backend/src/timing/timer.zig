const std = @import("std");

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