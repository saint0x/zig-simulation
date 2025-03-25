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