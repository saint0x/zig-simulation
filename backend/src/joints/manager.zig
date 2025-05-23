const std = @import("std");
const types = @import("core").types;
const timing = @import("timing");
const safety = @import("safety");
const kinematics = @import("kinematics");
const pid = @import("control").pid;

pub const JointManager = struct {
    allocator: std.mem.Allocator,
    joints: [types.NUM_JOINTS]types.JointState,
    configs: [types.NUM_JOINTS]types.JointConfig,
    controllers: [types.NUM_JOINTS]pid.PIDController,
    timing_system: *timing.TimingSystem,
    safety_monitor: *safety.SafetyMonitor,
    collision_detector: *kinematics.collision_detection.CollisionDetection,
    fk: *kinematics.ForwardKinematics,
    control_mode: types.ControlMode,

    pub fn init(
        allocator: std.mem.Allocator,
        timing_system: *timing.TimingSystem,
        safety_monitor: *safety.SafetyMonitor,
        collision_detector: *kinematics.collision_detection.CollisionDetection,
        fk: *kinematics.ForwardKinematics,
    ) !*JointManager {
        const self = try allocator.create(JointManager);
        self.* = .{
            .allocator = allocator,
            .joints = undefined,
            .configs = undefined,
            .controllers = undefined,
            .timing_system = timing_system,
            .safety_monitor = safety_monitor,
            .collision_detector = collision_detector,
            .fk = fk,
            .control_mode = types.ControlMode.position,
        };

        // Initialize joints with default configs
        for (&self.joints) |*joint| {
            joint.* = .{
                .current_angle = 0,
                .current_velocity = 0,
                .target_angle = 0,
                .target_velocity = 0,
                .current_torque = 0,
                .target_torque = 0,
                .temperature = 25.0,  // Room temperature
                .current = 0,
                .integral_term = 0,
                .last_error = 0,
            };
        }

        // Initialize configs with default values
        for (&self.configs, 0..) |*config, i| {
            config.* = .{
                .id = @enumFromInt(i),
                .min_angle = -180,
                .max_angle = 180,
                .max_velocity = 100,
                .max_acceleration = 50,
                .dt = 0.001,
                .pid_gains = .{
                    .kp = 1.0,
                    .ki = 0.1,
                    .kd = 0.01,
                    .i_max = 100.0,
                },
            };
        }

        // Initialize PID controllers
        for (&self.controllers, 0..) |*controller, i| {
            const initial_state = types.JointState{
                .current_angle = 0,
                .current_velocity = 0,
                .target_angle = 0,
                .target_velocity = 0,
                .current_torque = 0,
                .target_torque = 0,
                .temperature = 25.0,  // Room temperature
                .current = 0,
                .integral_term = 0,
                .last_error = 0,
            };
            controller.* = pid.PIDController.init(
                self.configs[i],
                initial_state,
            );
        }

        return self;
    }

    pub fn deinit(self: *JointManager) void {
        self.allocator.destroy(self);
    }

    pub fn updateStates(self: *JointManager) !void {
        for (&self.joints, 0..) |*joint, i| {
            const controller = &self.controllers[i];
            
            // Update joint state based on controller output
            joint.current_velocity = controller.update();
            joint.current_angle += joint.current_velocity * (1.0 / 1000.0); // 1kHz update rate
            
            // Check safety limits
            if (!self.safety_monitor.checkSafety(&[_]types.JointState{joint.*})) {
                return error.SafetyLimitExceeded;
            }
        }
    }

    pub fn getJointState(self: *JointManager, joint_id: types.JointId) types.JointState {
        return self.joints[@intFromEnum(joint_id)];
    }

    pub fn getRobotState(self: *JointManager) types.RobotState {
        // TODO: Implement actual logic to determine robot state
        _ = self; // Keep self until proper logic is added
        return types.RobotState.ready; // Placeholder
    }

    pub fn reset(self: *JointManager) void {
        for (&self.controllers) |*controller| {
            controller.reset();
        }
        for (&self.joints) |*joint| {
            joint.* = .{
                .current_angle = 0,
                .current_velocity = 0,
                .target_angle = 0,
                .target_velocity = 0,
                .current_torque = 0,
                .target_torque = 0,
                .temperature = 25.0,  // Room temperature
                .current = 0,
                .integral_term = 0,
                .last_error = 0,
            };
        }
        self.control_mode = types.ControlMode.position;
    }

    pub fn setTorques(self: *JointManager, torques: []const f32) !void {
        // TODO: Implement actual logic, possibly check control mode first
        if (torques.len != types.NUM_JOINTS) {
            return error.InvalidNumberOfTorques;
        }
        for (&self.joints, torques) |*joint, torque| {
            joint.target_torque = torque;
        }
        self.control_mode = types.ControlMode.position;
    }

    pub fn setControlMode(self: *JointManager, mode: types.ControlMode, params: anytype) !void {
        // TODO: Implement actual logic for switching control modes
        // params might contain stiffness, damping, etc. for certain modes
        _ = params;
        self.control_mode = mode;
        std.log.info("Set control mode to: {}", .{mode});
    }
}; 