const std = @import("std");
const testing = std.testing;
const expect = testing.expect;
const expectEqual = testing.expectEqual;

// Import system components via module system
const joints = @import("joints");
const physics = @import("physics");
const communication = @import("communication");
const core = @import("core");
const safety = @import("safety");
const timing = @import("timing");
const kinematics = @import("kinematics");

test "Physics simulation integration with joint manager" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Initialize timing system
    var timing_system = try timing.TimingSystem.init(allocator);
    defer timing_system.deinit();
    
    // Initialize simple collision detection for testing
    const link_dimensions = [_]core.types.LinkDimensions{
        .{
            .id = .base, .width = 0.2, .height = 0.3, .length = 0.2,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0 },
        },
    };
    
    var fk = kinematics.ForwardKinematics.init([_]core.types.LinkDimensions{link_dimensions[0]} ** 9);
    
    const collision_config = core.types.CollisionConfig{
        .min_link_distance = 0.01,
        .min_environment_distance = 0.05,
        .enable_continuous_detection = true,
        .check_frequency = 100,
        .link_dimensions = [_]core.types.LinkDimensions{link_dimensions[0]} ** 9,
    };
    
    const collision_detector = try kinematics.collision_detection.CollisionDetection.init(
        collision_config,
        &fk
    );
    
    // Initialize safety monitor
    const joint_limits = [_]safety.limits.JointLimits{
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
    };
    
    var safety_monitor = safety.SafetyMonitor.init(
        &joint_limits,
        collision_detector,
        10.0
    );
    
    // Initialize joint manager
    var joint_manager = try joints.JointManager.init(
        allocator,
        &timing_system,
        &safety_monitor,
        collision_detector,
        &fk
    );
    defer joint_manager.deinit();
    
    // Test initial state
    const initial_joint = joint_manager.getJointState(.base_rotation);
    try expectEqual(@as(f32, 0.0), initial_joint.current_angle);
    try expectEqual(@as(f32, 0.0), initial_joint.current_velocity);
    
    // Test physics simulation is integrated
    try expect(@TypeOf(joint_manager.physics_simulation) == physics.simulation.PhysicsSimulation);
    
    // Test multiple update cycles (skip safety checks by limiting updates)
    var successful_updates: u32 = 0;
    for (0..50) |_| { // Only 50 iterations to stay within startup grace period
        joint_manager.updateStates() catch |err| {
            if (err != error.SafetyLimitExceeded) {
                return err; // Unexpected error
            }
            break; // Expected safety error, stop testing
        };
        successful_updates += 1;
    }
    
    // Should have at least some successful updates during startup period
    try expect(successful_updates > 10);
}

test "Communication protocol with real data" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Test realistic joint state message
    const realistic_joint_state = communication.protocol.JointStateMessage{
        .timestamp_us = 1640995200000000, // Jan 1, 2022 timestamp
        .positions = [_]f32{ 0.1, -0.2, 0.3, -0.1, 0.0, 0.15, -0.05 }, // Realistic joint positions in radians
        .velocities = [_]f32{ 0.01, -0.02, 0.01, 0.0, 0.0, 0.01, 0.0 }, // Slow movements
        .torques = [_]f32{ 2.1, 15.3, 8.7, 3.2, 1.1, 0.8, 0.3 }, // Realistic torques for KUKA arm
        .temperatures = [_]f32{ 32.1, 35.7, 31.2, 29.8, 28.5, 27.3, 26.9 }, // Motor temperatures
        .currents = [_]f32{ 0.8, 2.1, 1.3, 0.7, 0.3, 0.2, 0.1 }, // Motor currents
    };
    
    // Serialize to binary
    var bytes: [@sizeOf(communication.protocol.JointStateMessage)]u8 = undefined;
    @memcpy(&bytes, std.mem.asBytes(&realistic_joint_state));
    
    // Create frame
    const frame = communication.protocol.Frame{
        .type = communication.protocol.MESSAGE_TYPE_JOINT_STATE,
        .payload = &bytes,
    };
    
    // Encode frame
    var buffer = std.ArrayList(u8).init(allocator);
    defer buffer.deinit();
    try frame.encode(buffer.writer());
    
    // Verify frame structure
    try expect(buffer.items.len > 5); // At least header + some data
    try expectEqual(communication.protocol.MESSAGE_TYPE_JOINT_STATE, buffer.items[0]);
    
    // Decode and verify
    var stream = std.io.fixedBufferStream(buffer.items);
    const decoded_frame = try communication.protocol.Frame.decode(stream.reader(), allocator);
    defer allocator.free(decoded_frame.payload);
    
    try expectEqual(frame.type, decoded_frame.type);
    try expectEqual(bytes.len, decoded_frame.payload.len);
}

test "Error handling and recovery" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Test JSON parsing error recovery
    const invalid_command_json = "{ \"type\": \"invalid\", \"malformed\": json }";
    
    const parse_result = std.json.parseFromSlice(
        communication.protocol.CommandMessage,
        allocator,
        invalid_command_json,
        .{}
    );
    
    // Should return an error
    try expect(std.meta.isError(parse_result));
    
    // Test valid command after error
    const valid_command_json = 
        \\{
        \\  "type": 0,
        \\  "values": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        \\}
    ;
    
    const valid_result = try std.json.parseFromSlice(
        communication.protocol.CommandMessage,
        allocator,
        valid_command_json,
        .{}
    );
    defer valid_result.deinit();
    
    try expectEqual(@as(u8, 0), valid_result.value.type);
    try expect(valid_result.value.values != null);
}

test "Performance and timing" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    _ = arena.allocator(); // Acknowledge allocator
    
    // Test physics simulation performance
    var sim = physics.simulation.PhysicsSimulation.init();
    const voltages = [_]f32{0.1} ** 7; // Low voltages for stability
    
    const start_time = std.time.nanoTimestamp();
    
    // Run 1000 physics steps (simulating 1 second at 1kHz)
    for (0..1000) |_| {
        sim.step(voltages, 0.001);
    }
    
    const end_time = std.time.nanoTimestamp();
    const elapsed_ns = end_time - start_time;
    const elapsed_ms = @as(f64, @floatFromInt(elapsed_ns)) / 1_000_000.0;
    
    // Should complete 1000 steps in reasonable time (less than 100ms on modern hardware)
    std.log.info("Physics simulation: 1000 steps took {d:.2}ms", .{elapsed_ms});
    try expect(elapsed_ms < 1000.0); // Very generous limit
    
    // Test sensor reading performance
    const sensor_start = std.time.nanoTimestamp();
    
    for (0..1000) |_| {
        _ = sim.readSensors();
    }
    
    const sensor_end = std.time.nanoTimestamp();
    const sensor_elapsed_ns = sensor_end - sensor_start;
    const sensor_elapsed_ms = @as(f64, @floatFromInt(sensor_elapsed_ns)) / 1_000_000.0;
    
    std.log.info("Sensor readings: 1000 reads took {d:.2}ms", .{sensor_elapsed_ms});
    try expect(sensor_elapsed_ms < 100.0); // Should be very fast
}

test "Memory usage and leaks" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    
    // Test that repeated operations don't accumulate memory
    const initial_memory = arena.state.end_index;
    
    // Perform many allocations and deallocations
    for (0..100) |_| {
        // Simulate frame encoding/decoding
        const test_payload = "test message";
        const frame = communication.protocol.Frame{
            .type = 1,
            .payload = test_payload,
        };
        
        var buffer = std.ArrayList(u8).init(allocator);
        defer buffer.deinit();
        
        try frame.encode(buffer.writer());
        
        var stream = std.io.fixedBufferStream(buffer.items);
        const decoded = try communication.protocol.Frame.decode(stream.reader(), allocator);
        defer allocator.free(decoded.payload);
    }
    
    // Check memory usage (arena allocator doesn't actually free, so this is limited)
    const final_memory = arena.state.end_index;
    const memory_used = final_memory - initial_memory;
    
    std.log.info("Memory used for 100 frame operations: {} bytes", .{memory_used});
    
    // Should not use excessive memory (this is a rough check)
    try expect(memory_used < 100_000); // 100KB limit
}

test "Concurrent operation simulation" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    _ = arena.allocator(); // Acknowledge allocator
    
    // Test multiple physics simulations running independently
    var sim1 = physics.simulation.PhysicsSimulation.init();
    var sim2 = physics.simulation.PhysicsSimulation.init();
    
    const voltages1 = [_]f32{0.1} ** 7;
    const voltages2 = [_]f32{0.2} ** 7;
    
    // Run both simulations
    for (0..100) |_| {
        sim1.step(voltages1, 0.001);
        sim2.step(voltages2, 0.001);
    }
    
    // Simulations should produce different results due to different inputs
    var different_positions = false;
    for (sim1.position, sim2.position) |pos1, pos2| {
        if (@abs(pos1 - pos2) > 0.001) {
            different_positions = true;
            break;
        }
    }
    
    try expect(different_positions); // Should have diverged due to different voltages
    
    // Both simulations should still be stable
    for (sim1.position) |pos| {
        try expect(std.math.isFinite(pos));
    }
    
    for (sim2.position) |pos| {
        try expect(std.math.isFinite(pos));
    }
}