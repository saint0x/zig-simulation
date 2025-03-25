const std = @import("std");
const types = @import("types.zig");

pub const PhysicsSimulation = struct {
    // Joint dynamics
    inertia: [types.NUM_JOINTS]f32,
    damping: [types.NUM_JOINTS]f32,
    friction: [types.NUM_JOINTS]f32,
    gravity_compensation: [types.NUM_JOINTS]f32,

    // State variables
    position: [types.NUM_JOINTS]f32,
    velocity: [types.NUM_JOINTS]f32,
    acceleration: [types.NUM_JOINTS]f32,
    
    // Motor characteristics
    motor_constants: [types.NUM_JOINTS]types.MotorConstants,
    thermal_model: [types.NUM_JOINTS]types.ThermalModel,
    electrical_model: [types.NUM_JOINTS]types.ElectricalModel,

    // Sensor simulation
    position_sensors: [types.NUM_JOINTS]types.EncoderSimulation,
    current_sensors: [types.NUM_JOINTS]types.CurrentSensorSimulation,
    temperature_sensors: [types.NUM_JOINTS]types.TemperatureSensorSimulation,

    pub fn init() PhysicsSimulation {
        const sim = PhysicsSimulation{
            .inertia = [_]f32{0.5} ** types.NUM_JOINTS,
            .damping = [_]f32{0.1} ** types.NUM_JOINTS,
            .friction = [_]f32{0.05} ** types.NUM_JOINTS,
            .gravity_compensation = [_]f32{0.0} ** types.NUM_JOINTS,
            .position = [_]f32{0.0} ** types.NUM_JOINTS,
            .velocity = [_]f32{0.0} ** types.NUM_JOINTS,
            .acceleration = [_]f32{0.0} ** types.NUM_JOINTS,
            .motor_constants = [_]types.MotorConstants{.{
                .kt = 0.1,
                .ke = 0.1,
                .resistance = 1.0,
                .inductance = 0.001,
                .max_current = 10.0,
                .max_temp = 85.0,
            }} ** types.NUM_JOINTS,
            .thermal_model = [_]types.ThermalModel{.{
                .thermal_resistance = 10.0,
                .thermal_capacitance = 10.0,
                .ambient_temp = 25.0,
                .current_temp = 25.0,
            }} ** types.NUM_JOINTS,
            .electrical_model = [_]types.ElectricalModel{.{
                .current = 0.0,
                .voltage = 0.0,
                .back_emf = 0.0,
            }} ** types.NUM_JOINTS,
            .position_sensors = [_]types.EncoderSimulation{.{
                .resolution = 4096.0,
                .position = 0.0,
                .noise_amplitude = 0.1,
            }} ** types.NUM_JOINTS,
            .current_sensors = [_]types.CurrentSensorSimulation{.{
                .resolution = 12.0,
                .range = 20.0,
                .noise_std = 0.01,
            }} ** types.NUM_JOINTS,
            .temperature_sensors = [_]types.TemperatureSensorSimulation{.{
                .response_time = 1.0,
                .noise_amplitude = 0.1,
                .filtered_temp = 25.0,
            }} ** types.NUM_JOINTS,
        };
        return sim;
    }

    pub fn step(self: *PhysicsSimulation, voltages: [types.NUM_JOINTS]f32, dt: f32) void {
        // Update each joint
        for (0..types.NUM_JOINTS) |i| {
            // Update electrical model
            self.electrical_model[i].updateCurrent(
                self.motor_constants[i],
                self.velocity[i],
                voltages[i],
                dt
            );

            // Calculate torque
            const motor_torque = self.motor_constants[i].kt * self.electrical_model[i].current;
            const friction_torque = -std.math.sign(self.velocity[i]) * self.friction[i];
            const damping_torque = -self.damping[i] * self.velocity[i];
            const gravity_torque = self.gravity_compensation[i];
            const net_torque = motor_torque + friction_torque + damping_torque + gravity_torque;

            // Update dynamics
            self.acceleration[i] = net_torque / self.inertia[i];
            self.velocity[i] += self.acceleration[i] * dt;
            self.position[i] += self.velocity[i] * dt;

            // Update thermal model
            const power_dissipated = self.electrical_model[i].current * self.electrical_model[i].current * 
                                   self.motor_constants[i].resistance;
            self.thermal_model[i].update(power_dissipated, dt);
        }
    }

    pub fn readSensors(self: *PhysicsSimulation) struct {
        positions: [types.NUM_JOINTS]f32,
        currents: [types.NUM_JOINTS]f32,
        temperatures: [types.NUM_JOINTS]f32,
    } {
        var readings = .{
            .positions = undefined,
            .currents = undefined,
            .temperatures = undefined,
        };

        for (0..types.NUM_JOINTS) |i| {
            readings.positions[i] = self.position_sensors[i].readPosition(self.position[i]);
            readings.currents[i] = self.current_sensors[i].readCurrent(self.electrical_model[i].current);
            readings.temperatures[i] = self.temperature_sensors[i].readTemperature(
                self.thermal_model[i].current_temp,
                0.001 // 1ms sample time
            );
        }

        return readings;
    }
}; 