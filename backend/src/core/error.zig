const std = @import("std");
const types = @import("types.zig");
const JointId = types.JointId;
const RobotStatus = types.RobotStatus;

/// Error types for the KUKA arm simulation
pub const Error = error{
    // Joint-related errors
    JointLimitViolation,
    JointVelocityExceeded,
    JointAccelerationExceeded,
    JointJerkExceeded,
    JointNotReady,
    JointFault,
    
    // Motion planning errors
    InvalidTrajectory,
    TrajectoryExecutionFailed,
    PathPlanningFailed,
    CollisionDetected,
    
    // Safety system errors
    EmergencyStopTriggered,
    SafetyLimitExceeded,
    SystemFault,
    
    // General errors
    InvalidState,
    InvalidCommand,
    CommunicationError,
    Timeout,
};

/// Error context with detailed information
pub const ErrorContext = struct {
    /// The specific error that occurred
    err: Error,
    /// Joint ID if the error is joint-related
    joint_id: ?JointId,
    /// Additional error message
    message: []const u8,
    /// Timestamp when the error occurred
    timestamp: i64,
    /// Current robot status
    robot_status: RobotStatus,
};

/// Error handler for the KUKA arm
pub const ErrorHandler = struct {
    /// Current error context if any
    current_error: ?ErrorContext,
    /// Error recovery state
    recovery_state: enum {
        none,
        recovering,
        recovered,
        failed,
    },
    /// Error history (circular buffer)
    error_history: std.BoundedArray(ErrorContext, 100),
    /// Allocator for error messages
    allocator: std.mem.Allocator,

    /// Initialize a new error handler
    pub fn init(allocator: std.mem.Allocator) ErrorHandler {
        return .{
            .current_error = null,
            .recovery_state = .none,
            .error_history = std.BoundedArray(ErrorContext, 100).init(0) catch unreachable,
            .allocator = allocator,
        };
    }

    /// Handle a new error
    pub fn handleError(self: *ErrorHandler, err: Error, joint_id: ?JointId, message: []const u8, robot_status: RobotStatus) void {
        const context = ErrorContext{
            .err = err,
            .joint_id = joint_id,
            .message = message,
            .timestamp = std.time.timestamp(),
            .robot_status = robot_status,
        };

        // Update current error
        self.current_error = context;

        // Add to history
        if (self.error_history.len >= 100) {
            _ = self.error_history.orderedRemove(0);
        }
        self.error_history.append(context) catch unreachable;

        // Set recovery state
        self.recovery_state = .recovering;
    }

    /// Attempt to recover from the current error
    pub fn recover(self: *ErrorHandler) bool {
        if (self.current_error) |err| {
            switch (err.err) {
                .JointLimitViolation, .JointVelocityExceeded, .JointAccelerationExceeded, .JointJerkExceeded => {
                    // These errors can be recovered from by stopping and resetting
                    self.recovery_state = .recovered;
                    self.current_error = null;
                    return true;
                },
                .EmergencyStopTriggered => {
                    // Emergency stop requires manual reset
                    self.recovery_state = .failed;
                    return false;
                },
                .CollisionDetected => {
                    // Collision requires path replanning
                    self.recovery_state = .recovered;
                    self.current_error = null;
                    return true;
                },
                else => {
                    // Other errors require more complex recovery
                    self.recovery_state = .failed;
                    return false;
                },
            }
        }
        return true;
    }

    /// Get the current error context
    pub fn getCurrentError(self: *const ErrorHandler) ?ErrorContext {
        return self.current_error;
    }

    /// Get the current recovery state
    pub fn getRecoveryState(self: *const ErrorHandler) enum {
        none,
        recovering,
        recovered,
        failed,
    } {
        return self.recovery_state;
    }

    /// Clear the current error
    pub fn clearError(self: *ErrorHandler) void {
        self.current_error = null;
        self.recovery_state = .none;
    }

    /// Get the error history
    pub fn getErrorHistory(self: *const ErrorHandler) []const ErrorContext {
        return self.error_history.constSlice();
    }
}; 