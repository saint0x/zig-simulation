//! By convention, root.zig is the root source file when making a library. If
//! you are making an executable, the convention is to delete this file and
//! start with main.zig instead.
const std = @import("std");
const testing = std.testing;

pub const core = @import("core");
pub const kinematics = @import("kinematics");
pub const safety = @import("safety");
pub const timing = @import("timing");
pub const hal = @import("hal");
pub const control = @import("control");
pub const physics = @import("physics");
pub const communication = @import("communication");
pub const utils = @import("utils");

pub const Error = error{
    OutOfMemory,
    ConnectionRefused,
    InvalidOperation,
    SafetyLimitExceeded,
    CollisionDetected,
    ConfigurationError,
    CommunicationError,
    Timeout,
    InitializationFailed,
    InternalError,
    // Added specific communication/command errors
    InvalidControlMode,
    InvalidCommandPayload,
    InvalidSafetyCommand,
    InvalidNumberOfTorques,
    MissingControlModeValue, // Added this one too
};

pub const Result = union(enum) {
    // ... existing code ...
};

pub export fn add(a: i32, b: i32) i32 {
    return a + b;
}

test "basic add functionality" {
    try testing.expect(add(3, 7) == 10);
}

test {
    // Run all tests
    std.testing.refAllDecls(@This());
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var core_instance = try core.init(allocator);
    defer core_instance.deinit();

    var kinematics_instance = try kinematics.init(allocator);
    defer kinematics_instance.deinit();

    var safety_instance = try safety.init(allocator);
    defer safety_instance.deinit();

    var timing_instance = try timing.init(allocator);
    defer timing_instance.deinit();

    var hal_instance = try hal.init(allocator);
    defer hal_instance.deinit();

    // Initialize control module with default joint configurations and timing system
    var control_instance = try control.init(null, &timing_instance);
    defer control_instance.deinit();

    try core_instance.run();
}
