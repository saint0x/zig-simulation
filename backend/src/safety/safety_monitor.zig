const std = @import("std");
const core = @import("core");
const kinematics = @import("kinematics");
const types = @import("types.zig");

const NUM_JOINTS = core.types.NUM_JOINTS;
const JointConfig = core.types.JointConfig;
const JointState = core.types.JointState;
const CollisionResult = kinematics.CollisionResult;

pub const SafetyMonitor = struct {
    joint_limits: *const [NUM_JOINTS]types.JointLimit,
    collision_detector: *kinematics.CollisionDetection,
    emergency_stop_threshold: f32,
    is_emergency_stop: bool,
    is_enabled: bool,

    pub fn init(joint_limits: *const [NUM_JOINTS]types.JointLimit, collision_detector: *kinematics.CollisionDetection, emergency_stop_threshold: f32) SafetyMonitor {
        return SafetyMonitor{
            .joint_limits = joint_limits,
            .collision_detector = collision_detector,
            .emergency_stop_threshold = emergency_stop_threshold,
            .is_emergency_stop = false,
            .is_enabled = true, // Safety is enabled by default
        };
    }

    /// Check all safety conditions. Returns true if safe, false if unsafe.
    /// When disabled, always returns true (bypasses safety checks).
    pub fn checkSafety(self: *SafetyMonitor, joint_states: []const JointState) bool {
        // If safety monitoring is disabled, always return safe
        if (!self.is_enabled) {
            return true;
        }

        // Emergency stop takes precedence
        if (self.is_emergency_stop) {
            return false;
        }

        // Check joint limits
        for (joint_states, 0..) |state, i| {
            const limit = self.joint_limits.*[i];
            if (state.current_angle < limit.min_angle or state.current_angle > limit.max_angle) {
                std.log.warn("Safety: Joint {} angle {d:.2}° exceeds limits [{d:.2}°, {d:.2}°]", .{ i, state.current_angle, limit.min_angle, limit.max_angle });
                self.is_emergency_stop = true;
                return false;
            }
            if (state.current_velocity < -limit.max_velocity or state.current_velocity > limit.max_velocity) {
                std.log.warn("Safety: Joint {} velocity {d:.2}°/s exceeds max {d:.2}°/s", .{ i, state.current_velocity, limit.max_velocity });
                self.is_emergency_stop = true;
                return false;
            }
        }

        // Check for collisions
        const current_time = @as(f64, @floatFromInt(std.time.nanoTimestamp()));
        const collision_result = self.collision_detector.checkCollisions(current_time);
        if (collision_result.collision_detected) {
            std.log.warn("Safety: Collision detected!", .{});
            self.is_emergency_stop = true;
            return false;
        }

        return true;
    }

    /// Returns true if emergency stop is active
    pub fn isEmergencyStop(self: *const SafetyMonitor) bool {
        return self.is_emergency_stop;
    }

    /// Returns true if safety monitoring is enabled
    pub fn isEnabled(self: *const SafetyMonitor) bool {
        return self.is_enabled;
    }

    /// Enable safety monitoring
    pub fn enable(self: *SafetyMonitor) void {
        self.is_enabled = true;
        std.log.info("Safety monitoring enabled", .{});
    }

    /// Disable safety monitoring (use with caution!)
    pub fn disable(self: *SafetyMonitor) void {
        self.is_enabled = false;
        std.log.warn("Safety monitoring DISABLED - use with caution!", .{});
    }

    /// Trigger emergency stop
    pub fn emergencyStop(self: *SafetyMonitor) void {
        self.is_emergency_stop = true;
        std.log.err("EMERGENCY STOP triggered!", .{});
    }

    /// Reset safety state (clears emergency stop, keeps enabled state)
    pub fn reset(self: *SafetyMonitor) void {
        self.is_emergency_stop = false;
        std.log.info("Safety state reset", .{});
    }
}; 