const std = @import("std");
const core = @import("core");
const control = @import("..");
const pid = control.pid;
const types = core.types;
const timing = @import("../../timing");
const safety = @import("safety");
const kinematics = @import("kinematics");

pub const JointConfig = core.types.JointConfig;
pub const JointState = core.types.JointState;
pub const JointId = core.types.JointId;
pub const RobotState = core.types.RobotState;
pub const NUM_JOINTS = types.NUM_JOINTS;
pub const PIDController = pid.PIDController;
pub const TimingSystem = timing.TimingSystem;
pub const SafetyMonitor = safety.SafetyMonitor;
pub const CollisionDetection = kinematics.collision_detection.CollisionDetection;
pub const ForwardKinematics = kinematics.ForwardKinematics;

/// Manages all joints of the robot arm
pub const JointManager = struct {
    /// Array of joint states
    joints: [core.types.NUM_JOINTS]core.types.JointState,
    /// Array of joint configurations
    configs: [core.types.NUM_JOINTS]core.types.JointConfig,
    /// Timing system for control loop
    timing_system: *core.TimingSystem,
    /// Safety monitor
    safety: core.SafetyMonitor,
    /// Collision detection
    collision_detector: *kinematics.collision_detection.CollisionDetection,
    /// Current robot state
    robot_state: core.types.RobotState,
    /// Error handler
    error_handler: core.error_handler.ErrorHandler,

    pub fn init(configs: ?[core.types.NUM_JOINTS]core.types.JointConfig, timing_system: *core.TimingSystem) !*JointManager {
        var joint_mgr = try std.heap.page_allocator.create(JointManager);
        
        // Initialize joint configurations
        if (configs) |joint_configs| {
            joint_mgr.configs = joint_configs;
        } else {
            // Default configurations
            for (0..core.types.NUM_JOINTS) |i| {
                joint_mgr.configs[i] = .{
                    .id = @enumFromInt(i),
                    .min_angle = -180,
                    .max_angle = 180,
                    .max_velocity = 90,
                    .max_acceleration = 180,
                    .dt = 0.001,
                    .pid_gains = .{
                        .kp = 1.0,
                        .ki = 0.1,
                        .kd = 0.01,
                        .i_max = 10.0,
                    },
                };
            }
        }

        // Initialize joint states
        for (0..core.types.NUM_JOINTS) |i| {
            joint_mgr.joints[i] = .{
                .current_angle = 0,
                .current_velocity = 0,
                .target_angle = 0,
                .target_velocity = 0,
                .integral_term = 0,
                .last_error = 0,
            };
        }

        // Initialize timing system
        joint_mgr.timing_system = timing_system;

        // Initialize collision detector
        joint_mgr.collision_detector = try kinematics.collision_detection.CollisionDetection.init(
            .{
                .min_link_distance = 0.1,
                .min_environment_distance = 0.2,
                .enable_continuous_detection = true,
                .check_frequency = 100,
                .link_dimensions = undefined, // TODO: Set actual link dimensions
            },
            undefined, // TODO: Set forward kinematics
        );

        // Initialize safety monitor
        var joint_limits: [core.types.NUM_JOINTS]safety.JointLimits = undefined;
        for (0..core.types.NUM_JOINTS) |i| {
            joint_limits[i] = safety.JointLimits.init(
                joint_mgr.configs[i].min_angle,
                joint_mgr.configs[i].max_angle,
            );
        }
        joint_mgr.safety = safety.SafetyMonitor.init(
            &joint_limits,
            joint_mgr.collision_detector,
            0.1, // Emergency stop threshold
        );

        // Initialize robot state
        joint_mgr.robot_state = .powered_off;

        // Initialize error handler
        joint_mgr.error_handler = core.error_handler.ErrorHandler.init(std.heap.page_allocator);

        return joint_mgr;
    }

    pub fn deinit(self: *JointManager) void {
        self.collision_detector.deinit();
        std.heap.page_allocator.destroy(self);
    }

    pub fn reset(self: *JointManager) void {
        // Reset joint states
        for (0..core.types.NUM_JOINTS) |i| {
            self.joints[i] = .{
                .current_angle = 0,
                .current_velocity = 0,
                .target_angle = 0,
                .target_velocity = 0,
                .integral_term = 0,
                .last_error = 0,
            };
        }

        // Reset robot state
        self.robot_state = .powered_off;

        // Reset error handler
        self.error_handler.clearError();
    }

    pub fn update(self: *JointManager) !void {
        // Skip update if robot is not ready
        if (self.robot_state != .ready) return;

        // Get current time
        const current_time = @as(f64, @floatFromInt(std.time.nanoTimestamp()));

        // Update each joint
        for (0..core.types.NUM_JOINTS) |i| {
            // Update joint state based on PID control
            const position_error = self.joints[i].target_angle - self.joints[i].current_angle;
            const dt = self.configs[i].dt;

            // Calculate PID terms
            const p_term = position_error * self.configs[i].pid_gains.kp;
            self.joints[i].integral_term += position_error * dt;
            if (self.joints[i].integral_term > self.configs[i].pid_gains.i_max) {
                self.joints[i].integral_term = self.configs[i].pid_gains.i_max;
            }
            const i_term = self.joints[i].integral_term * self.configs[i].pid_gains.ki;
            const d_term = (position_error - self.joints[i].last_error) / dt * self.configs[i].pid_gains.kd;

            // Calculate control output
            const control_output = p_term + i_term + d_term;

            // Update joint state
            self.joints[i].current_velocity = control_output;
            self.joints[i].current_angle += self.joints[i].current_velocity * dt;
            self.joints[i].last_error = position_error;

            // Check safety limits
            if (!self.safety.checkSafety(&[_]core.types.JointState{self.joints[i]})) {
                self.robot_state = .emergency_stop;
                return error.SafetyLimitExceeded;
            }
        }

        // Update collision detection
        const collision_result = self.collision_detector.checkCollisions(current_time);
        if (collision_result.collision_detected) {
            self.robot_state = .emergency_stop;
            return error.CollisionDetected;
        }
    }
}; 