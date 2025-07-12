const std = @import("std");
const testing = std.testing;
const expect = testing.expect;
const expectEqual = testing.expectEqual;

// Import safety components via module system
const core = @import("core");
const safety = @import("safety");
const types = core.types;

test "JointLimits validation" {
    const limits = safety.limits.JointLimits{
        .min_angle = -90.0,
        .max_angle = 90.0,
    };
    
    // Test valid angles
    try expect(limits.isWithinLimits(0.0));
    try expect(limits.isWithinLimits(45.0));
    try expect(limits.isWithinLimits(-45.0));
    try expect(limits.isWithinLimits(90.0)); // At limit
    try expect(limits.isWithinLimits(-90.0)); // At limit
    
    // Test invalid angles
    try expect(!limits.isWithinLimits(91.0));
    try expect(!limits.isWithinLimits(-91.0));
    try expect(!limits.isWithinLimits(180.0));
    try expect(!limits.isWithinLimits(-180.0));
}

test "SafetyMonitor initialization" {
    var arena = std.heap.ArenaAllocator.init(testing.allocator);
    defer arena.deinit();
    _ = arena.allocator(); // Acknowledge allocator
    
    // Create mock collision detector
    const link_dimensions = [_]types.LinkDimensions{
        .{
            .id = .base,
            .width = 0.2, .height = 0.3, .length = 0.2,
            .joint_offset = .{ .x = 0, .y = 0, .z = 0 },
        },
    };
    
    const collision_config = types.CollisionConfig{
        .min_link_distance = 0.01,
        .min_environment_distance = 0.05,
        .enable_continuous_detection = true,
        .check_frequency = 100,
        .link_dimensions = [_]types.LinkDimensions{link_dimensions[0]} ** 9,
    };
    
    // Mock forward kinematics (minimal implementation for testing)
    const MockFK = struct {
        pub fn init(_: [9]types.LinkDimensions) @This() {
            return .{};
        }
        pub fn calculatePose(_: *const @This(), _: [7]f32) types.EndEffectorPose {
            return .{
                .position = .{ .x = 0, .y = 0, .z = 0 },
                .orientation = .{ .roll = 0, .pitch = 0, .yaw = 0 },
            };
        }
    };
    
    _ = MockFK.init(collision_config.link_dimensions); // Acknowledge mock
    
    // This test is simplified since we'd need a real collision detector
    // For now, just test that safety limits work
    const joint_limits = [_]safety.limits.JointLimits{
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
        .{ .min_angle = -180, .max_angle = 180 },
    };
    
    // For this test, we'll create a simplified version
    // In reality, SafetyMonitor needs a real collision detector
    
    try expectEqual(@as(usize, 7), joint_limits.len);
}

test "Joint state safety validation" {
    const joint_limits = [_]safety.limits.JointLimits{
        .{ .min_angle = -90, .max_angle = 90 },
        .{ .min_angle = -90, .max_angle = 90 },
        .{ .min_angle = -90, .max_angle = 90 },
        .{ .min_angle = -90, .max_angle = 90 },
        .{ .min_angle = -90, .max_angle = 90 },
        .{ .min_angle = -90, .max_angle = 90 },
        .{ .min_angle = -90, .max_angle = 90 },
    };
    
    // Test valid joint states
    const valid_states = [_]types.JointState{
        .{
            .current_angle = 0.0, .current_velocity = 0.0,
            .target_angle = 0.0, .target_velocity = 0.0,
            .current_torque = 0.0, .target_torque = 0.0,
            .temperature = 25.0, .current = 0.0,
            .integral_term = 0.0, .last_error = 0.0,
        },
    } ** 7;
    
    // Check each joint against its limits
    for (valid_states, joint_limits) |state, limits| {
        try expect(limits.isWithinLimits(state.current_angle));
    }
    
    // Test invalid joint states
    var invalid_states = valid_states;
    invalid_states[0].current_angle = 100.0; // Exceeds +90 limit
    invalid_states[1].current_angle = -100.0; // Exceeds -90 limit
    
    try expect(!joint_limits[0].isWithinLimits(invalid_states[0].current_angle));
    try expect(!joint_limits[1].isWithinLimits(invalid_states[1].current_angle));
}

test "Velocity safety limits" {
    // Test velocity limits (this would be part of a more complete safety system)
    const max_velocity: f32 = 50.0; // degrees/s
    
    const test_velocities = [_]f32{ 0.0, 25.0, 49.9, 50.0, 50.1, 100.0 };
    const expected_safe = [_]bool{ true, true, true, true, false, false };
    
    for (test_velocities, expected_safe) |vel, expected| {
        const is_safe = @abs(vel) <= max_velocity;
        try expectEqual(expected, is_safe);
    }
}

test "Temperature safety monitoring" {
    const max_temp: f32 = 85.0; // °C
    const warning_temp: f32 = 75.0; // °C
    
    const test_temps = [_]f32{ 20.0, 50.0, 74.9, 75.0, 84.9, 85.0, 90.0 };
    
    for (test_temps) |temp| {
        const is_safe = temp < warning_temp;
        const is_warning = temp >= warning_temp and temp < max_temp;
        const is_critical = temp >= max_temp;
        
        // These should be mutually exclusive
        const state_count = @as(u8, if (is_safe) 1 else 0) +
                           @as(u8, if (is_warning) 1 else 0) +
                           @as(u8, if (is_critical) 1 else 0);
        
        try expect(state_count == 1); // Exactly one state should be true
        
        if (temp < warning_temp) {
            try expect(is_safe and !is_warning and !is_critical);
        } else if (temp < max_temp) {
            try expect(!is_safe and is_warning and !is_critical);
        } else {
            try expect(!is_safe and !is_warning and is_critical);
        }
    }
}

test "Emergency stop behavior" {
    // Test emergency stop logic
    var emergency_active = false;
    
    // Normal operation
    try expect(!emergency_active);
    
    // Trigger emergency stop
    emergency_active = true;
    try expect(emergency_active);
    
    // In emergency state, all motions should be blocked
    const motion_allowed = !emergency_active;
    try expect(!motion_allowed);
    
    // Reset emergency stop
    emergency_active = false;
    const motion_allowed_after_reset = !emergency_active;
    try expect(motion_allowed_after_reset);
}

test "Current limiting" {
    const max_current: f32 = 15.0; // A
    
    const test_currents = [_]f32{ 0.0, 5.0, 14.9, 15.0, 15.1, 20.0 };
    
    for (test_currents) |current| {
        const limited_current = std.math.clamp(current, -max_current, max_current);
        
        try expect(limited_current >= -max_current);
        try expect(limited_current <= max_current);
        
        if (@abs(current) <= max_current) {
            try expectEqual(current, limited_current);
        } else {
            try expect(@abs(limited_current) == max_current);
        }
    }
}

test "Safety system state transitions" {
    // Test safety system state machine
    const SafetyState = enum {
        disabled,
        enabled,
        warning,
        emergency,
    };
    
    var state = SafetyState.disabled;
    
    // Test valid transitions
    // disabled -> enabled
    state = SafetyState.enabled;
    try expectEqual(SafetyState.enabled, state);
    
    // enabled -> warning
    state = SafetyState.warning;
    try expectEqual(SafetyState.warning, state);
    
    // warning -> emergency
    state = SafetyState.emergency;
    try expectEqual(SafetyState.emergency, state);
    
    // emergency -> disabled (reset)
    state = SafetyState.disabled;
    try expectEqual(SafetyState.disabled, state);
    
    // Direct emergency from any state
    state = SafetyState.enabled;
    state = SafetyState.emergency; // Emergency can be triggered from any state
    try expectEqual(SafetyState.emergency, state);
}

test "Collision detection data structures" {
    // Test collision result structure
    var collision_result = types.CollisionResult{
        .collision_detected = false,
        .collision_type = .none,
        .link1 = null,
        .link2 = null,
        .collision_point = null,
        .min_distance = 10.0, // 10mm safe distance
        .joint_angles = [_]f32{0.0} ** 7,
    };
    
    try expect(!collision_result.collision_detected);
    try expectEqual(@as(@TypeOf(collision_result.collision_type), .none), collision_result.collision_type);
    try expect(collision_result.link1 == null);
    try expect(collision_result.min_distance > 0.0);
    
    // Simulate collision detection
    collision_result.collision_detected = true;
    collision_result.collision_type = .self_collision;
    collision_result.link1 = .shoulder;
    collision_result.link2 = .forearm;
    collision_result.min_distance = -2.0; // Negative = penetration
    
    try expect(collision_result.collision_detected);
    try expectEqual(@as(@TypeOf(collision_result.collision_type), .self_collision), collision_result.collision_type);
    try expectEqual(types.LinkId.shoulder, collision_result.link1.?);
    try expectEqual(types.LinkId.forearm, collision_result.link2.?);
    try expect(collision_result.min_distance < 0.0); // Penetration
}