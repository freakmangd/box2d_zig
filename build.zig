const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const box2dc_dep = b.dependency("box2dc", .{});

    const lib = b.addStaticLibrary(.{
        .name = "box2dc",
        .target = target,
        .optimize = optimize,
        .root_source_file = .{ .path = "src/extern.zig" },
    });

    lib.root_module.addCSourceFiles(.{
        .root = box2dc_dep.path("src"),
        .files = box2d_source_files,
    });
    lib.linkLibC();
    lib.addIncludePath(box2dc_dep.path("extern/simde"));
    lib.addIncludePath(box2dc_dep.path("src"));
    lib.addIncludePath(box2dc_dep.path("include"));

    lib.installHeadersDirectory(box2dc_dep.path("src").getPath(b), "box2d");
    lib.installHeadersDirectory(box2dc_dep.path("include/box2d").getPath(b), "box2d");

    b.installArtifact(lib);

    // ========== HELPER ==========

    const helper_mod = b.addModule("box2d_zig", .{
        .root_source_file = .{ .path = "src/init.zig" },
    });
    helper_mod.linkLibrary(lib);

    // ========== TESTS ==========

    const exe_unit_tests = b.addTest(.{
        .root_source_file = .{ .path = "src/tests.zig" },
        .target = target,
        .optimize = optimize,
    });
    exe_unit_tests.root_module.linkLibrary(lib);

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_exe_unit_tests.step);
}

const SourceList = []const []const u8;

const box2d_source_files: SourceList = &.{
    "aabb.c",
    "allocate.c",
    "array.c",
    "bitset.c",
    "block_allocator.c",
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
    "implementation.c",
    "island.c",
    "joint.c",
    "manifold.c",
    "math.c",
    "motor_joint.c",
    "mouse_joint.c",
    "pool.c",
    "prismatic_joint.c",
    "revolute_joint.c",
    "shape.c",
    "solver.c",
    "stack_allocator.c",
    "table.c",
    "timer.c",
    "types.c",
    "weld_joint.c",
    "wheel_joint.c",
    "world.c",
};
