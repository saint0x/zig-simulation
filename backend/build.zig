const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    // Create the utils module first since other modules depend on it
    const utils = b.addModule("utils", .{
        .root_source_file = b.path("src/utils/root.zig"),
    });

    // Create the timing module first since core depends on it
    const timing = b.addModule("timing", .{
        .root_source_file = b.path("src/timing/root.zig"),
    });
    timing.addImport("utils", utils);

    // Create the kinematics module first since control depends on it
    const kinematics = b.addModule("kinematics", .{
        .root_source_file = b.path("src/kinematics/root.zig"),
    });
    kinematics.addImport("utils", utils);

    // Create the safety module first since control depends on it
    const safety = b.addModule("safety", .{
        .root_source_file = b.path("src/safety/root.zig"),
    });
    safety.addImport("utils", utils);
    safety.addImport("kinematics", kinematics);

    // Create the control module with all its dependencies
    const control = b.addModule("control", .{
        .root_source_file = b.path("src/control/root.zig"),
    });
    control.addImport("utils", utils);
    control.addImport("kinematics", kinematics);
    control.addImport("safety", safety);

    // Create the core module with all its dependencies
    const core = b.addModule("core", .{
        .root_source_file = b.path("src/core/root.zig"),
    });
    core.addImport("utils", utils);
    core.addImport("timing", timing);
    core.addImport("safety", safety);
    core.addImport("kinematics", kinematics);
    core.addImport("control", control);

    // Create the joints module
    const joints = b.addModule("joints", .{
        .root_source_file = b.path("src/joints/root.zig"),
    });
    joints.addImport("utils", utils);
    joints.addImport("timing", timing);
    joints.addImport("safety", safety);
    joints.addImport("kinematics", kinematics);
    joints.addImport("control", control);
    joints.addImport("core", core);

    // Update other modules' dependencies to include core
    control.addImport("core", core);
    kinematics.addImport("core", core);
    safety.addImport("core", core);

    // Create the HAL module
    const hal = b.addModule("hal", .{
        .root_source_file = b.path("src/hal/root.zig"),
    });
    hal.addImport("core", core);
    hal.addImport("safety", safety);
    hal.addImport("timing", timing);
    hal.addImport("utils", utils);

    // Create the physics module
    const physics = b.addModule("physics", .{
        .root_source_file = b.path("src/physics/root.zig"),
    });
    physics.addImport("core", core);
    physics.addImport("utils", utils);
    
    // Add physics dependency to joints now that physics module is created
    joints.addImport("physics", physics);

    // Create the communication module
    const communication = b.addModule("communication", .{
        .root_source_file = b.path("src/communication/root.zig"),
    });
    communication.addImport("core", core);
    communication.addImport("utils", utils);
    communication.addImport("joints", joints);

    // Create the backend library
    const backend_lib = b.addStaticLibrary(.{
        .name = "backend",
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Add all modules as dependencies
    backend_lib.root_module.addImport("core", core);
    backend_lib.root_module.addImport("kinematics", kinematics);
    backend_lib.root_module.addImport("safety", safety);
    backend_lib.root_module.addImport("timing", timing);
    backend_lib.root_module.addImport("hal", hal);
    backend_lib.root_module.addImport("control", control);
    backend_lib.root_module.addImport("physics", physics);
    backend_lib.root_module.addImport("communication", communication);
    backend_lib.root_module.addImport("utils", utils);
    backend_lib.root_module.addImport("joints", joints);

    b.installArtifact(backend_lib);

    // Create the backend executable
    const backend_exe = b.addExecutable(.{
        .name = "backend",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Link against the library
    backend_exe.linkLibrary(backend_lib);

    // Add all modules as dependencies to the executable
    backend_exe.root_module.addImport("core", core);
    backend_exe.root_module.addImport("kinematics", kinematics);
    backend_exe.root_module.addImport("safety", safety);
    backend_exe.root_module.addImport("timing", timing);
    backend_exe.root_module.addImport("hal", hal);
    backend_exe.root_module.addImport("control", control);
    backend_exe.root_module.addImport("physics", physics);
    backend_exe.root_module.addImport("communication", communication);
    backend_exe.root_module.addImport("utils", utils);
    backend_exe.root_module.addImport("joints", joints);

    b.installArtifact(backend_exe);

    // Create a run step
    const run_cmd = b.addRunArtifact(backend_exe);
    run_cmd.step.dependOn(b.getInstallStep());

    const run_step = b.step("run", "Run the backend");
    run_step.dependOn(&run_cmd.step);

    // Create test targets
    const test_step = b.step("test", "Run all tests");
    
    // Physics tests
    const physics_tests = b.addTest(.{
        .name = "physics_tests",
        .root_source_file = b.path("tests/physics/simulation_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    physics_tests.root_module.addImport("core", core);
    physics_tests.root_module.addImport("utils", utils);
    physics_tests.root_module.addImport("physics", physics);
    
    const run_physics_tests = b.addRunArtifact(physics_tests);
    test_step.dependOn(&run_physics_tests.step);
    
    // Protocol tests
    const protocol_tests = b.addTest(.{
        .name = "protocol_tests", 
        .root_source_file = b.path("tests/communication/protocol_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    protocol_tests.root_module.addImport("core", core);
    protocol_tests.root_module.addImport("communication", communication);
    
    const run_protocol_tests = b.addRunArtifact(protocol_tests);
    test_step.dependOn(&run_protocol_tests.step);
    
    // Safety tests
    const safety_tests = b.addTest(.{
        .name = "safety_tests",
        .root_source_file = b.path("tests/safety/safety_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    safety_tests.root_module.addImport("core", core);
    safety_tests.root_module.addImport("safety", safety);
    safety_tests.root_module.addImport("utils", utils);
    safety_tests.root_module.addImport("kinematics", kinematics);
    
    const run_safety_tests = b.addRunArtifact(safety_tests);
    test_step.dependOn(&run_safety_tests.step);
    
    // Integration tests
    const integration_tests = b.addTest(.{
        .name = "integration_tests",
        .root_source_file = b.path("tests/integration/full_system_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    integration_tests.root_module.addImport("core", core);
    integration_tests.root_module.addImport("joints", joints);
    integration_tests.root_module.addImport("physics", physics);
    integration_tests.root_module.addImport("communication", communication);
    integration_tests.root_module.addImport("safety", safety);
    integration_tests.root_module.addImport("timing", timing);
    integration_tests.root_module.addImport("kinematics", kinematics);
    integration_tests.root_module.addImport("utils", utils);
    
    const run_integration_tests = b.addRunArtifact(integration_tests);
    test_step.dependOn(&run_integration_tests.step);
}
