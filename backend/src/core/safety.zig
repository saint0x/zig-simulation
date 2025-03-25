const std = @import("std");
const types = @import("types.zig");
const JointState = types.JointState;
const JointConfig = types.JointConfig;
const RobotStatus = types.RobotStatus;

/// Safety monitoring system for the KUKA arm
pub const SafetyMonitor = struct {
    /// Maximum allowed joint velocity (rad/s)
    max_velocity: f32,
    /// Maximum allowed joint acceleration (rad/s²)
    max_acceleration: f32,
    /// Maximum allowed joint jerk (rad/s³)
    max_jerk: f32,
    /// Current safety status
    status: RobotStatus,
    /// Emergency stop flag
    emergency_stop: bool,
    /// Last joint states for velocity/acceleration calculation
    last_states: [7]JointState,
    /// Time since last state update (s)
    last_update_time: f32,

    /// Initialize a new safety monitor
    pub fn init() SafetyMonitor {
        return .{
            .max_velocity = 2.0, // 2 rad/s
            .max_acceleration = 1.0, // 1 rad/s²
            .max_jerk = 0.5, // 0.5 rad/s³
            .status = RobotStatus.idle,
            .emergency_stop = false,
            .last_states = [_]JointState{undefined} ** 7,
            .last_update_time = 0.0,
        };
    }

    /// Update safety monitoring with new joint states
    pub fn update(self: *SafetyMonitor, current_states: []const JointState, dt: f32) void {
        if (self.emergency_stop) {
            self.status = RobotStatus.emergency_stop;
            return;
        }

        // Check velocities and accelerations
        for (current_states, 0..) |state, i| {
            const last_state = self.last_states[i];
            const velocity = @abs(state.velocity);
            const acceleration = @abs(state.acceleration);

            // Check velocity limits
            if (velocity > self.max_velocity) {
                self.status = RobotStatus.fault;
                return;
            }

            // Check acceleration limits
            if (acceleration > self.max_acceleration) {
                self.status = RobotStatus.fault;
                return;
            }

            // Check for sudden changes (jerk)
            if (dt > 0) {
                const jerk = @abs(acceleration - last_state.acceleration) / dt;
                if (jerk > self.max_jerk) {
                    self.status = RobotStatus.fault;
                    return;
                }
            }

            // Update last state
            self.last_states[i] = state;
        }

        self.last_update_time += dt;
        self.status = RobotStatus.idle;
    }

    /// Trigger emergency stop
    pub fn emergencyStop(self: *SafetyMonitor) void {
        self.emergency_stop = true;
        self.status = RobotStatus.emergency_stop;
    }

    /// Reset safety monitor
    pub fn reset(self: *SafetyMonitor) void {
        self.emergency_stop = false;
        self.status = RobotStatus.idle;
        self.last_update_time = 0.0;
    }

    /// Get current safety status
    pub fn getStatus(self: *const SafetyMonitor) RobotStatus {
        return self.status;
    }

    /// Check if system is in emergency stop
    pub fn isEmergencyStop(self: *const SafetyMonitor) bool {
        return self.emergency_stop;
    }
}; 