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

    pub fn init(joint_limits: *const [NUM_JOINTS]types.JointLimit, collision_detector: *kinematics.CollisionDetection, emergency_stop_threshold: f32) SafetyMonitor {
        return SafetyMonitor{
            .joint_limits = joint_limits,
            .collision_detector = collision_detector,
            .emergency_stop_threshold = emergency_stop_threshold,
            .is_emergency_stop = false,
        };
    }

    pub fn checkSafety(self: *SafetyMonitor, joint_states: []const JointState) bool {
        // Check joint limits
        for (joint_states, 0..) |state, i| {
            const limit = self.joint_limits.*[i];
            if (state.current_angle < limit.min_angle or state.current_angle > limit.max_angle) {
                self.is_emergency_stop = true;
                return false;
            }
            if (state.current_velocity < -limit.max_velocity or state.current_velocity > limit.max_velocity) {
                self.is_emergency_stop = true;
                return false;
            }
        }

        // Check for collisions
        const current_time = @as(f64, @floatFromInt(std.time.nanoTimestamp()));
        const collision_result = self.collision_detector.checkCollisions(current_time);
        if (collision_result.collision_detected) {
            self.is_emergency_stop = true;
            return false;
        }

        return true;
    }

    pub fn isEmergencyStop(self: *const SafetyMonitor) bool {
        return self.is_emergency_stop;
    }

    pub fn reset(self: *SafetyMonitor) void {
        self.is_emergency_stop = false;
    }
}; 