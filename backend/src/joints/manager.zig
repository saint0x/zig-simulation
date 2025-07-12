const std = @import("std");
const types = @import("core").types;
const timing = @import("timing");
const safety = @import("safety");
const kinematics = @import("kinematics");
const pid = @import("control").pid;
const physics = @import("physics");

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
    physics_simulation: physics.simulation.PhysicsSimulation,
    startup_counter: u32,

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
            .physics_simulation = physics.simulation.PhysicsSimulation.init(),
            .startup_counter = 0,
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
        const dt = 0.001; // 1kHz update rate (1ms)
        self.startup_counter += 1;
        
        // Calculate control voltages for each joint based on current control mode
        var voltages: [types.NUM_JOINTS]f32 = undefined;
        
        for (&self.joints, 0..) |*joint, i| {
            const controller = &self.controllers[i];
            
            switch (self.control_mode) {
                .position => {
                    // PID position control - output is velocity command
                    const velocity_cmd = controller.update();
                    joint.target_velocity = velocity_cmd;
                    
                    // Convert velocity command to motor voltage
                    // Simplified model: voltage proportional to desired velocity
                    voltages[i] = velocity_cmd * 2.0; // Scale factor for voltage
                },
                .velocity => {
                    // Direct velocity control
                    voltages[i] = joint.target_velocity * 2.0;
                },
                .torque => {
                    // Convert torque command to voltage using motor constants
                    const motor_kt = self.physics_simulation.motor_constants[i].kt;
                    const desired_current = joint.target_torque / motor_kt;
                    voltages[i] = desired_current * self.physics_simulation.motor_constants[i].resistance;
                },
            }
            
            // Limit voltage to reasonable range
            voltages[i] = std.math.clamp(voltages[i], -24.0, 24.0); // ±24V typical
        }
        
        // Step the physics simulation with calculated voltages
        self.physics_simulation.step(voltages, dt);
        
        // Read sensor values from physics simulation
        const sensor_readings = self.physics_simulation.readSensors();
        
        // Update joint states with physics simulation results
        for (&self.joints, 0..) |*joint, i| {
            // Convert from radians to degrees for consistency with existing code
            joint.current_angle = sensor_readings.positions[i] * 180.0 / std.math.pi;
            joint.current_velocity = self.physics_simulation.velocity[i] * 180.0 / std.math.pi;
            joint.current_torque = self.physics_simulation.motor_constants[i].kt * 
                                  self.physics_simulation.electrical_model[i].current;
            joint.temperature = sensor_readings.temperatures[i];
            joint.current = sensor_readings.currents[i];
            
            // Update controllers with new position feedback (convert back to radians)
            const position_rad = sensor_readings.positions[i];
            self.controllers[i].state.current_angle = position_rad * 180.0 / std.math.pi;
            self.controllers[i].state.current_velocity = self.physics_simulation.velocity[i] * 180.0 / std.math.pi;
        }
        
        // Skip safety checks during startup (first 100 iterations = 100ms)
        if (self.startup_counter > 100) {
            // Check safety limits with updated states
            if (!self.safety_monitor.checkSafety(&self.joints)) {
                // Log current joint states for debugging
                std.log.err("Safety limit exceeded! Joint states:", .{});
                for (self.joints, 0..) |joint, i| {
                    std.log.err("  Joint {}: angle={d:.2}°, velocity={d:.2}°/s", .{ i, joint.current_angle, joint.current_velocity });
                }
                return error.SafetyLimitExceeded;
            }
        } else if (self.startup_counter % 50 == 0) {
            // Log startup progress
            std.log.info("Startup progress: {}/100 iterations", .{self.startup_counter});
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
        
        // Reset physics simulation to initial state
        self.physics_simulation = physics.simulation.PhysicsSimulation.init();
        
        // Reset startup counter
        self.startup_counter = 0;
        
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