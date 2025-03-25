const std = @import("std");
const utils = @import("utils");
const safety = @import("safety");

pub const MotorConfig = struct {
    max_velocity: f32,
    max_acceleration: f32,
    max_torque: f32,
    position_limits: safety.JointLimits,
};

pub const MotorDriver = struct {
    config: MotorConfig,
    safety_monitor: *safety.SafetyMonitor,
    allocator: std.mem.Allocator,
    current_position: f32,
    current_velocity: f32,
    current_torque: f32,
    is_enabled: bool,

    pub fn init(allocator: std.mem.Allocator, config: MotorConfig, safety_monitor: *safety.SafetyMonitor) !*MotorDriver {
        const self = try allocator.create(MotorDriver);
        self.* = MotorDriver{
            .config = config,
            .safety_monitor = safety_monitor,
            .allocator = allocator,
            .current_position = 0,
            .current_velocity = 0,
            .current_torque = 0,
            .is_enabled = false,
        };
        return self;
    }

    pub fn deinit(self: *MotorDriver) void {
        self.allocator.destroy(self);
    }

    pub fn enable(self: *MotorDriver) !void {
        if (self.safety_monitor.checkSafety()) {
            self.is_enabled = true;
            utils.log(utils.LOG_LEVEL_INFO, "Motor enabled", .{});
        } else {
            return error.SafetyCheckFailed;
        }
    }

    pub fn disable(self: *MotorDriver) void {
        self.is_enabled = false;
        utils.log(utils.LOG_LEVEL_INFO, "Motor disabled", .{});
    }

    pub fn setPosition(self: *MotorDriver, position: f32, max_velocity: ?f32) !void {
        if (!self.is_enabled) return error.MotorDisabled;
        if (!self.safety_monitor.checkSafety()) return error.SafetyCheckFailed;

        // Apply position limits
        const limited_position = std.math.clamp(
            position,
            self.config.position_limits.min_angle,
            self.config.position_limits.max_angle
        );

        // Apply velocity limits
        const velocity_limit = max_velocity orelse self.config.max_velocity;
        const actual_velocity = std.math.min(velocity_limit, self.config.max_velocity);

        // TODO: Implement actual motor control
        self.current_position = limited_position;
        self.current_velocity = actual_velocity;
    }

    pub fn getCurrentPosition(self: *const MotorDriver) f32 {
        return self.current_position;
    }

    pub fn getCurrentVelocity(self: *const MotorDriver) f32 {
        return self.current_velocity;
    }

    pub fn getCurrentTorque(self: *const MotorDriver) f32 {
        return self.current_torque;
    }
}; 