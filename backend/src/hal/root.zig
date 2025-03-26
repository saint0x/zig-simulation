const std = @import("std");

// HAL initialization and system
const init_module = @import("init.zig");
pub const HalSystem = init_module.HalSystem;
pub const init = init_module.init;

// Hardware communication
pub const communication = @import("communication.zig");
pub const SerialComm = communication.SerialComm;
pub const UsbComm = communication.UsbComm;

// Motor control
pub const motor_module = @import("motor.zig");
pub const MotorDriver = motor_module.MotorDriver;
pub const MotorConfig = motor_module.MotorConfig;

// Sensor interfaces
pub const sensors = @import("sensors.zig");
pub const JointSensor = sensors.JointSensor;
pub const ForceSensor = sensors.ForceSensor;
pub const PositionSensor = sensors.PositionSensor;

const types = @import("core").types;
const safety = @import("safety");

pub const HardwareInterface = struct {
    comm: *communication.CommunicationInterface,
    motors: [types.NUM_JOINTS]*motor_module.MotorDriver,
    allocator: std.mem.Allocator,

    const Self = @This();

    pub fn init(
        allocator: std.mem.Allocator,
        comm_config: communication.CommConfig,
        motor_configs: [types.NUM_JOINTS]motor_module.MotorConfig,
        safety_monitor: *safety.SafetyMonitor,
    ) !*Self {
        const self = try allocator.create(Self);
        
        // Initialize communication interface
        self.comm = try communication.CommunicationInterface.init(allocator, comm_config);
        
        // Initialize motor drivers
        for (motor_configs, 0..) |config, i| {
            self.motors[i] = try motor_module.MotorDriver.init(allocator, config, safety_monitor);
        }
        
        self.allocator = allocator;
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.comm.deinit();
        for (self.motors) |m| {
            m.deinit();
        }
        self.allocator.destroy(self);
    }

    pub fn enableMotors(self: *Self) !void {
        for (self.motors) |m| {
            try m.enable();
        }
    }

    pub fn disableMotors(self: *Self) void {
        for (self.motors) |m| {
            m.disable();
        }
    }

    pub fn setJointPositions(self: *Self, positions: []const f32, max_velocities: ?[]const f32) !void {
        if (positions.len != types.NUM_JOINTS) return error.InvalidJointCount;
        if (max_velocities) |vels| {
            if (vels.len != types.NUM_JOINTS) return error.InvalidJointCount;
        }

        // Send command through communication interface
        try self.comm.sendJointCommand(positions);

        // Update motor states
        for (self.motors, 0..) |m, i| {
            const max_vel = if (max_velocities) |vels| vels[i] else null;
            try m.setPosition(positions[i], max_vel);
        }
    }

    pub fn getJointStates(self: *Self) !types.JointState {
        var state = types.JointState{
            .current_angle = 0,
            .current_velocity = 0,
            .target_angle = 0,
            .target_velocity = 0,
            .current_torque = 0,
            .temperature = 0,
            .current = 0,
            .integral_term = 0,
            .last_error = 0,
        };

        for (self.motors) |m| {
            state.current_angle = m.getCurrentPosition();
            state.current_velocity = m.getCurrentVelocity();
            state.current_torque = m.getCurrentTorque();
        }

        return state;
    }

    pub fn update(self: *Self) !void {
        // Send heartbeat
        try self.comm.sendHeartbeat();

        // Get feedback from virtual motors
        _ = try self.comm.receiveJointFeedback();
    }
}; 