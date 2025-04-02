const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const box2d_dep = b.dependency("box2d", .{});

    const lib = b.addStaticLibrary(.{
        .name = "box2dc",
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/export.zig"),
    });

    lib.root_module.addCSourceFiles(.{
        .root = box2d_dep.path("src"),
        .files = box2d_source_files,
        .flags = &.{ "-Wno-psabi", "-mavx2" },
    });
    lib.linkLibC();
    lib.addIncludePath(box2d_dep.path("src"));
    lib.addIncludePath(box2d_dep.path("include"));

    lib.installHeadersDirectory(box2d_dep.path("src"), "box2d", .{});
    lib.installHeadersDirectory(box2d_dep.path("include/box2d"), "box2d", .{});

    b.installArtifact(lib);

    // ========== HELPER ==========

    const helper_mod = b.addModule("root", .{
        .root_source_file = b.path("src/init.zig"),
        .target = target,
        .optimize = optimize,
    });
    helper_mod.linkLibrary(lib);

    // ========== TESTS ==========

    const tests_mod = b.createModule(.{
        .root_source_file = b.path("src/tests.zig"),
        .target = target,
        .optimize = optimize,
    });

    const exe_unit_tests = b.addTest(.{ .root_module = tests_mod });
    exe_unit_tests.root_module.linkLibrary(lib);

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_exe_unit_tests.step);
}

const SourceList = []const []const u8;

const box2d_source_files: SourceList = &.{
    "aabb.c",
    "arena_allocator.c",
    "array.c",
    "bitset.c",
    "body.c",
    "broad_phase.c",
    "constraint_graph.c",
    "contact.c",
    "contact_solver.c",
    "core.c",
    "distance.c",
    "distance_joint.c",
    "dynamic_tree.c",
    "geometry.c",
    "hull.c",
    "id_pool.c",
    "island.c",
    "joint.c",
    "manifold.c",
    "math_functions.c",
    "motor_joint.c",
    "mouse_joint.c",
    "prismatic_joint.c",
    "revolute_joint.c",
    "sensor.c",
    "shape.c",
    "solver.c",
    "solver_set.c",
    "table.c",
    "timer.c",
    "types.c",
    "weld_joint.c",
    "wheel_joint.c",
    "world.c",
};
