const std = @import("std");

pub const c = @cImport({
    @cInclude("box2d/box2d.h");
    @cInclude("box2d/math.h");
    @cInclude("box2d/aabb.h");
    @cInclude("box2d/timer.h");
    @cInclude("box2d/bitset.h");
    @cInclude("box2d/geometry.h");
    @cInclude("box2d/types.h");
    @cInclude("box2d/distance.h");
    @cInclude("box2d/hull.h");
});

fn b2ComputeCircleMass(shape: [*c]const c.b2Circle, density: f32) callconv(.C) c.b2MassData {
    const rr: f32 = shape.*.radius * shape.*.radius;

    var massData: c.b2MassData = undefined;
    massData.mass = density * c.b2_pi * rr;
    massData.center = shape.*.point;

    // inertia about the local origin
    massData.I = massData.mass *
        (0.5 * rr + c.b2Dot(shape.*.point, shape.*.point));

    return massData;
}

test "shape mass" {
    const circle: c.b2Circle = .{
        .point = .{ .x = 1, .y = 0 },
        .radius = 1,
    };

    const md = b2ComputeCircleMass(&circle, 1.0);
    try std.testing.expectEqual(1, md.center.x);
    try std.testing.expectEqual(0, md.center.y);
    try std.testing.expectApproxEqAbs(0, md.I - 1.5 * c.b2_pi, @sqrt(std.math.floatEps(f32)));
}
