const std = @import("std");

/// Timing configuration for the control system
pub const TimingConfig = struct {
    /// Control loop frequency in Hz (1000 Hz = 1kHz)
    control_frequency: u32 = 1000,
    /// Maximum allowed jitter in microseconds
    max_jitter: u32 = 100,
    /// Minimum allowed jitter in microseconds
    min_jitter: u32 = 0,
};

/// Timing state and control
pub const TimingSystem = struct {
    config: TimingConfig,
    last_tick: i64,
    tick_count: u64,
    jitter_history: [100]u32,
    jitter_index: usize,

    const Self = @This();

    /// Initialize the timing system
    pub fn init(config: TimingConfig) Self {
        return .{
            .config = config,
            .last_tick = std.time.microTimestamp(),
            .tick_count = 0,
            .jitter_history = [_]u32{0} ** 100,
            .jitter_index = 0,
        };
    }

    /// Wait for the next control loop tick
    /// Returns true if timing requirements are met, false if there was too much jitter
    pub fn waitForNextTick(self: *Self) i64 {
        // Calculate expected time between ticks in microseconds
        const expected = @divTrunc(@as(i64, 1000000), @as(i64, self.config.control_frequency));
        
        // Calculate next tick time
        const next_tick = self.last_tick + expected;
        
        // Get current time
        const current = std.time.microTimestamp();
        
        // Calculate time to wait
        const wait_time = next_tick - current;
        
        // Sleep if needed
        if (wait_time > 0) {
            const wait_time_f = @as(f64, @floatFromInt(wait_time));
            const sleep_ns = @as(u64, @intFromFloat(wait_time_f * 1000.0));
            std.time.sleep(sleep_ns);
        }
        
        // Update last tick time
        self.last_tick = std.time.microTimestamp();
        
        // Calculate jitter
        const jitter = self.last_tick - next_tick;
        
        // Update jitter statistics
        const abs_jitter = @as(u32, @intCast(if (jitter < 0) -jitter else jitter));
        if (abs_jitter > self.config.max_jitter) {
            self.config.max_jitter = abs_jitter;
        }
        if (abs_jitter < self.config.min_jitter) {
            self.config.min_jitter = abs_jitter;
        }
        
        return jitter;
    }

    /// Get the current jitter statistics
    pub fn getJitterStats(self: *const Self) struct { avg: f32, max: u32 } {
        var sum: u64 = 0;
        var max_jitter: u32 = 0;

        for (self.jitter_history) |jitter| {
            sum += jitter;
            max_jitter = @max(max_jitter, jitter);
        }

        return .{
            .avg = @as(f32, @floatFromInt(sum)) / 100.0,
            .max = max_jitter,
        };
    }

    /// Get the current tick count
    pub fn getTickCount(self: *const Self) u64 {
        return self.tick_count;
    }
}; 