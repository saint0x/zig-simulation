const std = @import("std");
const types = @import("types.zig");
const pid = @import("pid.zig");
const timing = @import("timing.zig");
const safety = @import("safety.zig");

const JointConfig = types.JointConfig;
const JointState = types.JointState;
const JointId = types.JointId;
const RobotState = types.RobotState;
const NUM_JOINTS = types.NUM_JOINTS;
const PIDController = pid.PIDController;
const TimingSystem = timing.TimingSystem;
const SafetyMonitor = safety.SafetyMonitor;

/// Default joint configurations for the KUKA arm
pub const default_joint_configs = [NUM_JOINTS]JointConfig{
    // Base rotation
    .{
        .id = .base_rotation,
        .min_angle = -180,
        .max_angle = 180,
        .max_velocity = 120,
        .max_acceleration = 180,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 2.0,
            .ki = 0.1,
            .kd = 0.5,
            .i_max = 10.0,
        },
    },
    // Shoulder rotation
    .{
        .id = .shoulder_rotation,
        .min_angle = -90,
        .max_angle = 90,
        .max_velocity = 90,
        .max_acceleration = 135,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 2.5,
            .ki = 0.15,
            .kd = 0.6,
            .i_max = 8.0,
        },
    },
    // Elbow rotation
    .{
        .id = .elbow_rotation,
        .min_angle = -120,
        .max_angle = 120,
        .max_velocity = 150,
        .max_acceleration = 225,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 2.0,
            .ki = 0.1,
            .kd = 0.4,
            .i_max = 12.0,
        },
    },
    // Wrist bend
    .{
        .id = .wrist_bend,
        .min_angle = -135,
        .max_angle = 135,
        .max_velocity = 180,
        .max_acceleration = 270,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 1.8,
            .ki = 0.08,
            .kd = 0.35,
            .i_max = 15.0,
        },
    },
    // Wrist rotation
    .{
        .id = .wrist_rotation,
        .min_angle = -180,
        .max_angle = 180,
        .max_velocity = 200,
        .max_acceleration = 300,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 1.5,
            .ki = 0.05,
            .kd = 0.3,
            .i_max = 20.0,
        },
    },
    // Tool rotation
    .{
        .id = .tool_rotation,
        .min_angle = -360,
        .max_angle = 360,
        .max_velocity = 250,
        .max_acceleration = 375,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 1.2,
            .ki = 0.03,
            .kd = 0.25,
            .i_max = 25.0,
        },
    },
    // Gripper
    .{
        .id = .gripper,
        .min_angle = -45,
        .max_angle = 45,
        .max_velocity = 90,
        .max_acceleration = 135,
        .dt = 0.001, // 1kHz control loop
        .pid_gains = .{
            .kp = 3.0,
            .ki = 0.2,
            .kd = 0.7,
            .i_max = 5.0,
        },
    },
};

/// Manages all joints of the robot arm
pub const JointManager = struct {
    /// Array of joint configurations
    configs: [NUM_JOINTS]JointConfig,
    /// Array of joint states
    states: [NUM_JOINTS]JointState,
    /// Array of PID controllers
    controllers: [NUM_JOINTS]PIDController,
    /// Current robot state
    robot_state: RobotState,
    /// Timing system for control loop
    timing: *TimingSystem,
    /// Safety monitor
    safety: SafetyMonitor,

    const Self = @This();

    /// Initialize a new joint manager with default or custom configurations
    pub fn init(configs: ?[NUM_JOINTS]JointConfig, timing_sys: *TimingSystem) Self {
        var states: [NUM_JOINTS]JointState = undefined;
        var controllers: [NUM_JOINTS]PIDController = undefined;
        const final_configs = configs orelse default_joint_configs;

        // Initialize each joint
        for (0..NUM_JOINTS) |i| {
            // Initialize joint state
            states[i] = .{
                .current_angle = 0,
                .current_velocity = 0,
                .target_angle = 0,
                .target_velocity = 0,
                .integral_term = 0,
                .last_error = 0,
            };

            // Initialize PID controller
            controllers[i] = PIDController.init(final_configs[i], states[i]);
        }

        return .{
            .configs = final_configs,
            .states = states,
            .controllers = controllers,
            .robot_state = .powered_off,
            .timing = timing_sys,
            .safety = SafetyMonitor.init(final_configs),
        };
    }

    /// Update all joints in the control loop
    pub fn update(self: *Self) void {
        // Skip update if robot is not ready
        if (self.robot_state != .ready and self.robot_state != .moving) {
            return;
        }

        // Update safety monitor
        const dt = 1.0 / @as(f32, @floatFromInt(self.timing.config.control_frequency));
        self.safety.update(&self.states, dt);

        // Check safety status
        switch (self.safety.getStatus()) {
            .fault => {
                self.robot_state = .fault;
                return;
            },
            .emergency_stop => {
                self.robot_state = .emergency_stop;
                return;
            },
            else => {},
        }

        // Update each joint
        for (0..NUM_JOINTS) |i| {
            // Update PID controller and get control signal
            // This will be used when we implement the hardware interface
            _ = self.controllers[i].update();

            // Update joint state
            self.states[i] = self.controllers[i].state;
        }
    }

    /// Set target positions for all joints
    pub fn setTargets(self: *Self, target_angles: [NUM_JOINTS]f32, target_velocities: [NUM_JOINTS]f32) void {
        // Check if robot is ready
        if (self.robot_state != .ready and self.robot_state != .moving) {
            return;
        }

        // Check if target angles are within limits
        if (!self.safety.checkJointAngles(&target_angles)) {
            self.robot_state = .fault;
            return;
        }

        // Update each joint's target
        for (0..NUM_JOINTS) |i| {
            // Clamp target angle to joint limits
            const clamped_angle = std.math.clamp(
                target_angles[i],
                self.configs[i].min_angle,
                self.configs[i].max_angle
            );
            
            // Clamp target velocity to joint limits
            const clamped_velocity = std.math.clamp(
                target_velocities[i],
                -self.configs[i].max_velocity,
                self.configs[i].max_velocity
            );

            self.controllers[i].setTarget(clamped_angle, clamped_velocity);
        }

        // Update robot state to moving
        self.robot_state = .moving;
    }

    /// Update current joint states
    pub fn updateStates(self: *Self, current_angles: [NUM_JOINTS]f32, current_velocities: [NUM_JOINTS]f32) void {
        // Check if current angles are within limits
        if (!self.safety.checkJointAngles(&current_angles)) {
            self.robot_state = .fault;
            return;
        }

        // Update each joint's state
        for (0..NUM_JOINTS) |i| {
            self.controllers[i].updateState(current_angles[i], current_velocities[i]);
        }
    }

    /// Reset all joints to their initial state
    pub fn reset(self: *Self) void {
        for (0..NUM_JOINTS) |i| {
            self.controllers[i].reset();
            self.states[i] = self.controllers[i].state;
        }
        self.safety.reset();
        self.robot_state = .ready;
    }

    /// Get the current state of a specific joint
    pub fn getJointState(self: *const Self, joint_id: JointId) JointState {
        return self.states[@intFromEnum(joint_id)];
    }

    /// Get the current robot state
    pub fn getRobotState(self: *const Self) RobotState {
        return self.robot_state;
    }

    /// Check if all joints have reached their target positions
    pub fn hasReachedTarget(self: *const Self) bool {
        for (0..NUM_JOINTS) |i| {
            const angle_error = @abs(
                self.states[i].target_angle - self.states[i].current_angle
            );
            if (angle_error > 0.1) { // 0.1 degree tolerance
                return false;
            }
        }
        return true;
    }
}; 