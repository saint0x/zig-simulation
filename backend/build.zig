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

    // Create the core module
    const core = b.addModule("core", .{
        .root_source_file = b.path("src/core/root.zig"),
    });

    // Create the timing module
    const timing = b.addModule("timing", .{
        .root_source_file = b.path("src/timing/root.zig"),
    });
    timing.addImport("core", core);

    // Create the kinematics module
    const kinematics = b.addModule("kinematics", .{
        .root_source_file = b.path("src/kinematics/root.zig"),
    });
    kinematics.addImport("core", core);

    // Create the safety module
    const safety = b.addModule("safety", .{
        .root_source_file = b.path("src/safety/root.zig"),
    });
    safety.addImport("core", core);
    safety.addImport("kinematics", kinematics);

    // Create the control module
    const control = b.addModule("control", .{
        .root_source_file = b.path("src/control/root.zig"),
    });
    control.addImport("core", core);
    control.addImport("kinematics", kinematics);
    control.addImport("safety", safety);
    control.addImport("timing", timing);

    // Add dependencies to core
    core.addImport("timing", timing);
    core.addImport("kinematics", kinematics);
    core.addImport("safety", safety);
    core.addImport("control", control);

    // Create the backend library
    const backend_lib = b.addStaticLibrary(.{
        .name = "backend",
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });
    backend_lib.root_module.addImport("core", core);
    backend_lib.root_module.addImport("kinematics", kinematics);
    backend_lib.root_module.addImport("safety", safety);
    backend_lib.root_module.addImport("timing", timing);
    backend_lib.root_module.addImport("control", control);
    b.installArtifact(backend_lib);

    // Create the backend executable
    const backend_exe = b.addExecutable(.{
        .name = "backend",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    backend_exe.root_module.addImport("core", core);
    backend_exe.root_module.addImport("kinematics", kinematics);
    backend_exe.root_module.addImport("safety", safety);
    backend_exe.root_module.addImport("timing", timing);
    backend_exe.root_module.addImport("control", control);
    backend_exe.linkLibrary(backend_lib);
    b.installArtifact(backend_exe);

    // Create a run step
    const run_cmd = b.addRunArtifact(backend_exe);
    run_cmd.step.dependOn(b.getInstallStep());

    const run_step = b.step("run", "Run the backend");
    run_step.dependOn(&run_cmd.step);
}
