const std = @import("std");

/// Number of joints in the KUKA arm (use same as core types)
const core = @import("core");
pub const NUM_JOINTS = core.types.NUM_JOINTS;

pub const MotorConstants = struct {
    kt: f32, // Torque constant (Nm/A)
    ke: f32, // Back-EMF constant (V/rad/s)
    resistance: f32, // Winding resistance (Ohm)
    inductance: f32, // Winding inductance (H)
    max_current: f32, // Maximum continuous current (A)
    max_temp: f32, // Maximum temperature (°C)
};

pub const ThermalModel = struct {
    thermal_resistance: f32, // °C/W
    thermal_capacitance: f32, // J/°C
    ambient_temp: f32, // °C
    current_temp: f32, // Current temperature
    
    pub fn update(self: *ThermalModel, power_dissipated: f32, dt: f32) void {
        const temp_diff = self.current_temp - self.ambient_temp;
        const cooling_power = temp_diff / self.thermal_resistance;
        const net_power = power_dissipated - cooling_power;
        self.current_temp += (net_power / self.thermal_capacitance) * dt;
    }
};

pub const ElectricalModel = struct {
    current: f32, // Current through motor (A)
    voltage: f32, // Applied voltage (V)
    back_emf: f32, // Back EMF voltage (V)
    
    pub fn updateCurrent(self: *ElectricalModel, motor: MotorConstants, velocity: f32, voltage: f32, dt: f32) void {
        const back_emf = motor.ke * velocity;
        const di_dt = (voltage - back_emf - motor.resistance * self.current) / motor.inductance;
        self.current += di_dt * dt;
        self.current = std.math.clamp(self.current, -motor.max_current, motor.max_current);
        self.voltage = voltage;
        self.back_emf = back_emf;
    }
};

pub const EncoderSimulation = struct {
    resolution: f32, // Ticks per revolution
    position: f32, // Current position in ticks
    noise_amplitude: f32, // Maximum noise amplitude
    
    pub fn readPosition(self: *EncoderSimulation, true_position: f32) f32 {
        const ticks = true_position * self.resolution / (2.0 * std.math.pi);
        // Simple deterministic noise based on time for repeatability
        const noise = @sin(true_position * 1000.0) * self.noise_amplitude;
        const quantized_ticks = @round(ticks + noise);
        return quantized_ticks * 2.0 * std.math.pi / self.resolution;
    }
};

pub const CurrentSensorSimulation = struct {
    resolution: f32, // ADC resolution in bits
    range: f32, // Maximum measurable current
    noise_std: f32, // Standard deviation of measurement noise
    
    pub fn readCurrent(self: *CurrentSensorSimulation, true_current: f32) f32 {
        // Simple deterministic noise for repeatability
        const noise = @sin(true_current * 500.0) * self.noise_std;
        const adc_steps = std.math.pow(f32, 2, self.resolution);
        const quantized_current = @round((true_current + noise) * adc_steps / self.range);
        return quantized_current * self.range / adc_steps;
    }
};

pub const TemperatureSensorSimulation = struct {
    response_time: f32, // Sensor response time constant
    noise_amplitude: f32, // Maximum noise amplitude
    filtered_temp: f32, // Filtered temperature reading
    
    pub fn readTemperature(self: *TemperatureSensorSimulation, true_temp: f32, dt: f32) f32 {
        const alpha = 1.0 - std.math.exp(-dt / self.response_time);
        self.filtered_temp += alpha * (true_temp - self.filtered_temp);
        // Simple deterministic noise for repeatability
        const noise = @sin(true_temp * 100.0) * self.noise_amplitude;
        return self.filtered_temp + noise;
    }
}; 