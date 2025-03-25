const std = @import("std");
const types = @import("types.zig");

const JointConfig = types.JointConfig;
const JointState = types.JointState;

/// PID Controller for joint control
pub const PIDController = struct {
    /// Joint configuration containing PID gains
    config: JointConfig,
    /// Current joint state
    state: JointState,
    /// Time step in seconds (1/1000 for 1kHz control loop)
    dt: f32 = 0.001,

    const Self = @This();

    /// Initialize a new PID controller
    pub fn init(config: JointConfig, initial_state: JointState) Self {
        return .{
            .config = config,
            .state = initial_state,
            .dt = 0.001, // 1kHz control loop
        };
    }

    /// Update the PID controller and return the control output
    /// Returns the control signal (torque command) for the joint
    pub fn update(self: *Self) f32 {
        // Calculate position error
        const position_error = self.state.target_angle - self.state.current_angle;

        // Update integral term with anti-windup
        if (@abs(position_error) < self.config.pid_gains.i_max) {
            self.state.integral_term += position_error;
        }

        // Calculate derivative term
        const derivative = (position_error - self.state.last_error) / self.config.dt;

        // Calculate control output
        const output = self.config.pid_gains.kp * position_error +
            self.config.pid_gains.ki * self.state.integral_term +
            self.config.pid_gains.kd * derivative;

        // Update state
        self.state.last_error = position_error;

        // Apply output limits
        return std.math.clamp(output, -self.config.max_velocity, self.config.max_velocity);
    }

    /// Reset the PID controller state
    pub fn reset(self: *Self) void {
        self.state.integral_term = 0;
        self.state.last_error = 0;
    }

    /// Update the target position
    pub fn setTarget(self: *Self, target_angle: f32, target_velocity: f32) void {
        // Clamp target angle to joint limits
        const clamped_angle = std.math.clamp(
            target_angle,
            self.config.min_angle,
            self.config.max_angle,
        );

        self.state.target_angle = clamped_angle;
        self.state.target_velocity = target_velocity;
    }

    /// Update the current joint state
    pub fn updateState(self: *Self, current_angle: f32, current_velocity: f32) void {
        self.state.current_angle = current_angle;
        self.state.current_velocity = current_velocity;
    }
}; 