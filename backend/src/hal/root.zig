pub const communication = @import("communication.zig");
pub const motor_driver = @import("motor_driver.zig");

const std = @import("std");
const types = @import("core").types;
const safety = @import("safety");

pub const HardwareInterface = struct {
    comm: *communication.CommunicationInterface,
    motors: [types.NUM_JOINTS]*motor_driver.MotorDriver,
    allocator: std.mem.Allocator,

    const Self = @This();

    pub fn init(
        allocator: std.mem.Allocator,
        comm_config: communication.CommConfig,
        motor_configs: [types.NUM_JOINTS]motor_driver.MotorConfig,
        safety_monitor: *safety.SafetyMonitor,
    ) !*Self {
        const self = try allocator.create(Self);
        
        // Initialize communication interface
        self.comm = try communication.CommunicationInterface.init(allocator, comm_config);
        
        // Initialize motor drivers
        for (motor_configs, 0..) |config, i| {
            self.motors[i] = try motor_driver.MotorDriver.init(allocator, config, safety_monitor);
        }
        
        self.allocator = allocator;
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.comm.deinit();
        for (self.motors) |motor| {
            motor.deinit();
        }
        self.allocator.destroy(self);
    }

    pub fn enableMotors(self: *Self) !void {
        for (self.motors) |motor| {
            try motor.enable();
        }
    }

    pub fn disableMotors(self: *Self) void {
        for (self.motors) |motor| {
            motor.disable();
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
        for (self.motors, 0..) |motor, i| {
            const max_vel = if (max_velocities) |vels| vels[i] else null;
            try motor.setPosition(positions[i], max_vel);
        }
    }

    pub fn getJointStates(self: *Self) !types.JointState {
        var state = types.JointState{
            .position = [_]f32{0} ** types.NUM_JOINTS,
            .velocity = [_]f32{0} ** types.NUM_JOINTS,
            .torque = [_]f32{0} ** types.NUM_JOINTS,
        };

        for (self.motors, 0..) |motor, i| {
            state.position[i] = motor.getCurrentPosition();
            state.velocity[i] = motor.getCurrentVelocity();
            state.torque[i] = motor.getCurrentTorque();
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