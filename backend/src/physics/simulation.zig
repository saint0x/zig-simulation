const std = @import("std");
const types = @import("types.zig");
const core = @import("core");

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
        // Parameters based on typical KUKA KR6 R900 robot (7 joints)
        const sim = PhysicsSimulation{
            // Inertia increases for joints further from base
            .inertia = [_]f32{ 2.5, 2.0, 1.5, 0.8, 0.4, 0.2, 0.1 },
            // Damping coefficients for smooth motion
            .damping = [_]f32{ 0.8, 0.7, 0.6, 0.4, 0.3, 0.2, 0.1 },
            // Static + dynamic friction
            .friction = [_]f32{ 0.15, 0.12, 0.1, 0.08, 0.05, 0.03, 0.02 },
            // Gravity compensation (more effect on middle joints)
            .gravity_compensation = [_]f32{ 0.0, 9.81, 4.905, 2.45, 0.0, 0.0, 0.0 },
            // Initialize to safe mid-range positions (0 degrees)
            .position = [_]f32{0.0} ** types.NUM_JOINTS,
            .velocity = [_]f32{0.0} ** types.NUM_JOINTS,
            .acceleration = [_]f32{0.0} ** types.NUM_JOINTS,
            .motor_constants = [_]types.MotorConstants{.{
                .kt = 0.12, // Nm/A
                .ke = 0.12, // V/rad/s (usually equal to kt)
                .resistance = 0.5, // Ohm
                .inductance = 0.001, // H
                .max_current = 15.0, // A
                .max_temp = 85.0, // °C
            }} ** types.NUM_JOINTS,
            .thermal_model = [_]types.ThermalModel{.{
                .thermal_resistance = 8.0, // °C/W
                .thermal_capacitance = 15.0, // J/°C
                .ambient_temp = 25.0,
                .current_temp = 25.0,
            }} ** types.NUM_JOINTS,
            .electrical_model = [_]types.ElectricalModel{.{
                .current = 0.0,
                .voltage = 0.0,
                .back_emf = 0.0,
            }} ** types.NUM_JOINTS,
            .position_sensors = [_]types.EncoderSimulation{.{
                .resolution = 8192.0, // 13-bit encoder
                .position = 0.0,
                .noise_amplitude = 0.0001, // Very small noise for high precision
            }} ** types.NUM_JOINTS,
            .current_sensors = [_]types.CurrentSensorSimulation{.{
                .resolution = 12.0, // 12-bit ADC
                .range = 30.0, // ±30A range
                .noise_std = 0.005, // 0.5% noise
            }} ** types.NUM_JOINTS,
            .temperature_sensors = [_]types.TemperatureSensorSimulation{.{
                .response_time = 2.0, // 2s response time
                .noise_amplitude = 0.05, // ±0.05°C noise
                .filtered_temp = 25.0,
            }} ** types.NUM_JOINTS,
        };
        return sim;
    }

    pub fn step(self: *PhysicsSimulation, voltages: [types.NUM_JOINTS]f32, dt: f32) void {
        // Update each joint
        for (0..types.NUM_JOINTS) |i| {
            // Update electrical model with current limiting
            self.electrical_model[i].updateCurrent(
                self.motor_constants[i],
                self.velocity[i],
                voltages[i],
                dt
            );

            // Calculate torques with non-linear effects
            const motor_torque = self.motor_constants[i].kt * self.electrical_model[i].current;
            
            // Coulomb + viscous friction model
            const coulomb_friction = self.friction[i];
            const viscous_friction = self.damping[i] * self.velocity[i];
            const friction_torque = if (self.velocity[i] == 0.0)
                std.math.clamp(motor_torque, -coulomb_friction, coulomb_friction)
            else
                -std.math.sign(self.velocity[i]) * (coulomb_friction + @abs(viscous_friction));

            // Position-dependent gravity compensation
            const gravity_torque = self.gravity_compensation[i] * 
                @cos(self.position[i]); // Simplified gravity model

            const net_torque = motor_torque + friction_torque + gravity_torque;

            // Update dynamics with acceleration limiting
            const max_accel = 100.0; // rad/s²
            self.acceleration[i] = std.math.clamp(
                net_torque / self.inertia[i],
                -max_accel,
                max_accel
            );
            
            // Update velocity with limits
            const max_vel = 3.14159; // ±π rad/s
            self.velocity[i] = std.math.clamp(
                self.velocity[i] + self.acceleration[i] * dt,
                -max_vel,
                max_vel
            );
            
            // Update position
            self.position[i] += self.velocity[i] * dt;

            // Wrap position to ±2π
            self.position[i] = @mod(
                self.position[i] + std.math.pi,
                2.0 * std.math.pi
            ) - std.math.pi;

            // Update thermal model with current-based heating
            const power_dissipated = self.electrical_model[i].current * self.electrical_model[i].current * 
                                   self.motor_constants[i].resistance;
            self.thermal_model[i].update(power_dissipated, dt);
        }
    }

    const SensorReadings = struct {
        positions: [types.NUM_JOINTS]f32,
        currents: [types.NUM_JOINTS]f32,
        temperatures: [types.NUM_JOINTS]f32,
    };
    
    pub fn readSensors(self: *PhysicsSimulation) SensorReadings {
        var readings: SensorReadings = undefined;

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