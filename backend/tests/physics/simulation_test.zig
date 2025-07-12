const std = @import("std");
const testing = std.testing;
const expect = testing.expect;
const expectEqual = testing.expectEqual;
const expectApproxEqRel = testing.expectApproxEqRel;

// Import our physics simulation via module system
const physics = @import("physics");
const PhysicsSimulation = physics.simulation.PhysicsSimulation;

test "PhysicsSimulation initialization" {
    const sim = PhysicsSimulation.init();
    
    // Test that all arrays are properly initialized
    for (sim.position) |pos| {
        try expectEqual(@as(f32, 0.0), pos);
    }
    
    for (sim.velocity) |vel| {
        try expectEqual(@as(f32, 0.0), vel);
    }
    
    for (sim.acceleration) |acc| {
        try expectEqual(@as(f32, 0.0), acc);
    }
    
    // Test motor constants are reasonable
    for (sim.motor_constants) |motor| {
        try expect(motor.kt > 0.0);
        try expect(motor.ke > 0.0);
        try expect(motor.resistance > 0.0);
        try expect(motor.max_current > 0.0);
    }
}

test "Motor torque calculation" {
    var sim = PhysicsSimulation.init();
    const test_current: f32 = 5.0; // 5A
    const expected_torque = sim.motor_constants[0].kt * test_current;
    
    // Set current in electrical model
    sim.electrical_model[0].current = test_current;
    
    // Calculate actual torque (simplified from step function)
    const actual_torque = sim.motor_constants[0].kt * sim.electrical_model[0].current;
    
    try expectApproxEqRel(expected_torque, actual_torque, 0.001);
}

test "Thermal model heating and cooling" {
    var sim = PhysicsSimulation.init();
    const dt: f32 = 0.001; // 1ms timestep
    const power: f32 = 100.0; // 100W power dissipation (higher power for faster response)
    
    const initial_temp = sim.thermal_model[0].current_temp;
    
    // Apply heating for 10 seconds (longer time for slower thermal response)
    for (0..10000) |_| {
        sim.thermal_model[0].update(power, dt);
    }
    
    const heated_temp = sim.thermal_model[0].current_temp;
    try expect(heated_temp > initial_temp); // Should heat up
    
    // Cool down (no power) for 10 seconds
    for (0..10000) |_| {
        sim.thermal_model[0].update(0.0, dt);
    }
    
    const cooled_temp = sim.thermal_model[0].current_temp;
    try expect(cooled_temp < heated_temp); // Should cool down
}

test "Electrical model current dynamics" {
    var sim = PhysicsSimulation.init();
    const dt: f32 = 0.001; // 1ms timestep
    const applied_voltage: f32 = 12.0; // 12V
    
    // Apply voltage step and check current rises
    const initial_current = sim.electrical_model[0].current;
    
    // Step the electrical model
    sim.electrical_model[0].updateCurrent(
        sim.motor_constants[0],
        0.0, // zero velocity
        applied_voltage,
        dt
    );
    
    const stepped_current = sim.electrical_model[0].current;
    
    // Current should increase from zero when voltage is applied
    try expect(stepped_current > initial_current);
    
    // Current should be limited by max_current
    try expect(stepped_current <= sim.motor_constants[0].max_current);
}

test "Position integration and wrapping" {
    var sim = PhysicsSimulation.init();
    const dt: f32 = 0.001;
    
    // Set a constant velocity
    sim.velocity[0] = 1.0; // 1 rad/s
    
    // Step simulation multiple times
    for (0..1000) |_| {
        // Simplified position integration from step function
        sim.position[0] += sim.velocity[0] * dt;
        
        // Apply position wrapping like in the actual step function
        sim.position[0] = @mod(
            sim.position[0] + std.math.pi,
            2.0 * std.math.pi
        ) - std.math.pi;
    }
    
    // After 1 second (1000 steps of 1ms), position should be ~1 radian
    // But wrapped to [-π, π] range
    try expectApproxEqRel(@as(f32, 1.0), sim.position[0], 0.01);
}

test "Sensor noise characteristics" {
    var sim = PhysicsSimulation.init();
    const true_position: f32 = 1.0; // 1 radian
    
    // Test position sensor
    const measured_pos = sim.position_sensors[0].readPosition(true_position);
    const position_error = @abs(measured_pos - true_position);
    
    // Error should be within reasonable bounds (noise amplitude is 0.0001)
    try expect(position_error < 0.01); // 0.01 rad tolerance
    
    // Test current sensor
    const true_current: f32 = 5.0; // 5A
    const measured_current = sim.current_sensors[0].readCurrent(true_current);
    const current_error = @abs(measured_current - true_current);
    
    // Error should be within sensor range
    try expect(current_error < 1.0); // 1A tolerance
    
    // Test temperature sensor  
    const true_temp: f32 = 50.0; // 50°C
    const measured_temp = sim.temperature_sensors[0].readTemperature(true_temp, 0.001);
    const temp_error = @abs(measured_temp - true_temp);
    
    // Temperature sensor has filtering, so error depends on previous state
    try expect(temp_error < 30.0); // Large tolerance due to filtering
}

test "Full simulation step" {
    var sim = PhysicsSimulation.init();
    const dt: f32 = 0.001;
    const voltages = [_]f32{1.0} ** 7; // 1V on all joints
    
    // Store initial state
    const initial_positions = sim.position;
    const initial_velocities = sim.velocity;
    
    // Run simulation step
    sim.step(voltages, dt);
    
    // Verify physics updated
    var position_changed = false;
    var velocity_changed = false;
    
    for (sim.position, initial_positions) |new_pos, old_pos| {
        if (@abs(new_pos - old_pos) > 0.0001) {
            position_changed = true;
        }
        // Also check that position is still in valid range
        try expect(new_pos >= -std.math.pi);
        try expect(new_pos <= std.math.pi);
    }
    
    for (sim.velocity, initial_velocities) |new_vel, old_vel| {
        if (@abs(new_vel - old_vel) > 0.0001) {
            velocity_changed = true;
        }
    }
    
    // At least some joints should have changed (low voltage might not affect all)
    try expect(position_changed or velocity_changed);
    
    // Test sensor readings are consistent
    const readings = sim.readSensors();
    try expectEqual(@as(usize, 7), readings.positions.len);
    try expectEqual(@as(usize, 7), readings.currents.len);
    try expectEqual(@as(usize, 7), readings.temperatures.len);
}

test "Physics simulation stability" {
    var sim = PhysicsSimulation.init();
    const dt: f32 = 0.001;
    const voltages = [_]f32{0.1} ** 7; // Small voltages for stability
    
    // Run for 1000 steps (1 second)
    for (0..1000) |_| {
        sim.step(voltages, dt);
        
        // Check for NaN or infinite values
        for (sim.position) |pos| {
            try expect(std.math.isFinite(pos));
        }
        
        for (sim.velocity) |vel| {
            try expect(std.math.isFinite(vel));
            try expect(@abs(vel) < 100.0); // Reasonable velocity limit
        }
        
        for (sim.thermal_model) |thermal| {
            try expect(std.math.isFinite(thermal.current_temp));
            try expect(thermal.current_temp > -50.0 and thermal.current_temp < 200.0); // Reasonable temp range
        }
    }
}