const std = @import("std");

// Import all test modules
const physics_tests = @import("physics/simulation_test.zig");
const protocol_tests = @import("communication/protocol_test.zig");
const safety_tests = @import("safety/safety_test.zig");
const integration_tests = @import("integration/full_system_test.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    
    std.log.info("Running KUKA Backend Test Suite", .{});
    std.log.info("=================================", .{});
    
    // Run all tests (this is a simple test runner)
    std.log.info("All tests must be run via 'zig test' command", .{});
    std.log.info("Example: zig test tests/physics/simulation_test.zig", .{});
}

// This allows running individual test files
test {
    // Import all test files to make them discoverable
    _ = physics_tests;
    _ = protocol_tests;
    _ = safety_tests;
    _ = integration_tests;
}