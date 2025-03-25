const std = @import("std");
const utils = @import("utils");
const types = @import("types.zig");

// Timing initialization and system
const init_module = @import("init.zig");
pub const TimingSystem = types.TimingSystem;
pub const init = init_module.init;

// Timer functionality
pub const timer = @import("timer.zig");
pub const Timer = timer.Timer;

// Time utilities
pub const time_utils = @import("time_utils.zig");
pub const TimeConversion = time_utils.TimeConversion;

// Scheduling
pub const scheduler = @import("scheduler.zig");
pub const TaskScheduler = scheduler.TaskScheduler;
pub const Task = scheduler.Task;
pub const TaskPriority = scheduler.TaskPriority;

// Re-export core types
pub const TimeUnit = types.TimeUnit;
pub const TimerConfig = types.TimerConfig;
pub const TimingConfig = types.TimingConfig;

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

pub fn start(self: *Timer, current_time: i64) void {
    self.start_time = current_time;
}

pub fn hasElapsed(self: *const Timer, current_time: i64) bool {
    return (current_time - self.start_time) >= self.duration;
} 