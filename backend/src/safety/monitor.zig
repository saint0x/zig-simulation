const std = @import("std");
const utils = @import("utils");
const types = @import("types.zig");
const kinematics = @import("kinematics");
const core = @import("core");

pub const SafetyMonitor = struct {
    joint_limits: *const [core.types.NUM_JOINTS]types.JointLimit,
    collision_detector: *kinematics.collision_detection.CollisionDetection,
    emergency_stop_threshold: f32,
    enabled: bool,
    e_stop_active: bool,

    pub fn init(
        joint_limits: *const [core.types.NUM_JOINTS]types.JointLimit,
        collision_detector: *kinematics.collision_detection.CollisionDetection,
        emergency_stop_threshold: f32,
    ) SafetyMonitor {
        std.log.info("SafetyMonitor initialized with limits:", .{});
        for (joint_limits.*, 0..) |limit, i| {
            std.log.info("  Joint {}: velocity limit = {d:.1}°/s", .{i, limit.max_velocity});
        }
        return SafetyMonitor{
            .joint_limits = joint_limits,
            .collision_detector = collision_detector,
            .emergency_stop_threshold = emergency_stop_threshold,
            .enabled = true,
            .e_stop_active = false,
        };
    }
    
    pub fn enable(self: *SafetyMonitor) void {
        self.enabled = true;
        self.e_stop_active = false;
        std.log.info("SafetyMonitor enabled", .{});
    }

    pub fn disable(self: *SafetyMonitor) void {
        self.enabled = false;
        std.log.info("SafetyMonitor disabled", .{});
    }

    pub fn reset(self: *SafetyMonitor) void {
        self.enable();
        std.log.info("SafetyMonitor reset", .{});
    }

    pub fn emergencyStop(self: *SafetyMonitor) void {
        self.e_stop_active = true;
        self.enabled = false;
        std.log.info("SafetyMonitor emergency stop activated", .{});
    }

    pub fn checkSafety(self: *SafetyMonitor, joint_states: []const core.types.JointState) bool {
        if (!self.enabled or self.e_stop_active) {
            return false;
        }
        // Check joint limits
        for (joint_states, 0..) |state, i| {
            const limit = self.joint_limits.*[i];
            if (state.current_angle < limit.min_angle or state.current_angle > limit.max_angle) {
                std.log.err("Joint {} angle limit exceeded: {} not in [{}, {}]", .{i, state.current_angle, limit.min_angle, limit.max_angle});
                return false;
            }
            // Check velocity limits 
            const abs_velocity = @abs(state.current_velocity);
            if (abs_velocity > limit.max_velocity) {
                std.log.err("Joint {} velocity limit exceeded: {d:.2}°/s > {d:.2}°/s (limit)", .{i, abs_velocity, limit.max_velocity});
                return false;
            }
        }

        // Check for collisions with relaxed settings
        const current_time = @as(f64, @floatFromInt(std.time.nanoTimestamp()));
        const collision_result = self.collision_detector.checkCollisions(current_time);
        if (collision_result.collision_detected and collision_result.min_distance < -0.1) {
            // Only trigger on significant penetration (>10cm) to avoid false positives
            return false;
        }

        return true;
    }
}; 