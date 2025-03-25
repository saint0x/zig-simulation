pub const types = @import("types.zig");
pub const simulation = @import("simulation.zig");

test "physics-simulation" {
    const std = @import("std");
    const testing = std.testing;
    
    // Create simulation
    var sim = simulation.PhysicsSimulation.init();
    
    // Test initial conditions
    try testing.expectEqual(@as(f32, 0.0), sim.position[0]);
    try testing.expectEqual(@as(f32, 0.0), sim.velocity[0]);
    
    // Apply voltage and step simulation
    const voltages = [_]f32{12.0} ** types.NUM_JOINTS;
    const dt = 0.001;
    
    // Run for 100 steps
    var i: usize = 0;
    while (i < 100) : (i += 1) {
        sim.step(voltages, dt);
    }
    
    // Check that motor moved
    try testing.expect(sim.position[0] > 0.0);
    try testing.expect(sim.velocity[0] > 0.0);
    
    // Check sensor readings
    const readings = sim.readSensors();
    try testing.expect(readings.positions[0] > 0.0);
    try testing.expect(readings.currents[0] > 0.0);
    try testing.expect(readings.temperatures[0] > 25.0);
} 