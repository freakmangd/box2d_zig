const std = @import("std");
const b2 = @import("init.zig");
const c = b2.c;

const epsilon = @sqrt(std.math.floatEps(f32));

test "bitset" {
    const count = 169;

    var bitSet = c.b2CreateBitSet(count);
    defer c.b2DestroyBitSet(&bitSet);

    c.b2SetBitCountAndClear(&bitSet, count);
    var values: [count]bool = .{false} ** count;

    var num1: u32 = 0;
    var num2: u32 = 1;
    c.b2SetBit(&bitSet, num1);
    values[num1] = true;

    while (num2 < count) {
        c.b2SetBit(&bitSet, num2);
        values[num2] = true;
        const next = num1 + num2;
        num1 = num2;
        num2 = next;
    }

    for (0..count) |i| {
        const value = c.b2GetBit(&bitSet, @intCast(i));
        try std.testing.expect(value == values[i]);
    }
}

test "collision" {
    var a = b2.aabb(-1, -1, -2, -2);
    try std.testing.expect(c.b2AABB_IsValid(a) == false);

    a.upperBound = b2.vec2(1, 1);
    try std.testing.expect(c.b2AABB_IsValid(a) == true);

    const b = b2.aabb(2, 2, 4, 4);
    try std.testing.expect(c.b2AABB_Overlaps(a, b) == false);
    try std.testing.expect(c.b2AABB_Contains(a, b) == false);

    const p1 = b2.vec2(-2, 0);
    const p2 = b2.vec2(2, 0);

    const output = c.b2AABB_RayCast(a, p1, p2);
    try std.testing.expect(output.hit == true);
    try std.testing.expect(0.1 < output.fraction and output.fraction < 0.9);
}

test "segment distance" {
    const p1: c.b2Vec2 = b2.vec2(-1, -1);
    const q1: c.b2Vec2 = b2.vec2(-1, 1);
    const p2: c.b2Vec2 = b2.vec2(2, 0);
    const q2: c.b2Vec2 = b2.vec2(1, 0);

    const result = c.b2SegmentDistance(p1, q1, p2, q2);

    try std.testing.expectApproxEqAbs(0, result.fraction1 - 0.5, epsilon);
    try std.testing.expectApproxEqAbs(0, result.fraction2 - 1.0, epsilon);
    try std.testing.expectApproxEqAbs(0, result.closest1.x + 1.0, epsilon);
    try std.testing.expectApproxEqAbs(0, result.closest1.y, epsilon);
    try std.testing.expectApproxEqAbs(0, result.closest2.x - 1.0, epsilon);
    try std.testing.expectApproxEqAbs(0, result.closest2.y, epsilon);
    try std.testing.expectApproxEqAbs(0, result.distanceSquared - 4.0, epsilon);
}

test "shape distance" {
    var vas = [_]c.b2Vec2{
        b2.vec2(-1, -1),
        b2.vec2(1, -1),
        b2.vec2(1, 1),
        b2.vec2(-1, 1),
    };
    var vbs = [_]c.b2Vec2{
        b2.vec2(2, -1),
        b2.vec2(2, 1),
    };

    var input: c.b2DistanceInput = .{
        .proxyA = c.b2MakeProxy(&vas, vas.len, 0.0),
        .proxyB = c.b2MakeProxy(&vbs, vbs.len, 0.0),
        .transformA = c.b2Transform_identity,
        .transformB = c.b2Transform_identity,
        .useRadii = false,
    };

    var cache: c.b2DistanceCache = .{};
    const output = c.b2ShapeDistance(&cache, &input);

    try std.testing.expectApproxEqAbs(0, output.distance - 1.0, epsilon);
}

test "shape cast" {
    var vas = [_]c.b2Vec2{
        b2.vec2(-1, -1),
        b2.vec2(1, -1),
        b2.vec2(1, 1),
        b2.vec2(-1, 1),
    };
    var vbs = [_]c.b2Vec2{
        b2.vec2(2, -1),
        b2.vec2(2, 1),
    };

    var input: c.b2ShapeCastPairInput = .{
        .proxyA = c.b2MakeProxy(&vas, vas.len, 0.0),
        .proxyB = c.b2MakeProxy(&vbs, vbs.len, 0.0),
        .transformA = c.b2Transform_identity,
        .transformB = c.b2Transform_identity,
        .translationB = b2.vec2(-2, 0),
        .maxFraction = 1.0,
    };
    const output = c.b2ShapeCast(&input);

    try std.testing.expect(output.hit);
    try std.testing.expectApproxEqAbs(0, output.fraction - 0.5, c.b2_linearSlop);
}

test "time of impact" {
    var vas = [_]c.b2Vec2{
        b2.vec2(-1, -1),
        b2.vec2(1, -1),
        b2.vec2(1, 1),
        b2.vec2(-1, 1),
    };
    var vbs = [_]c.b2Vec2{
        b2.vec2(2, -1),
        b2.vec2(2, 1),
    };

    var input: c.b2TOIInput = .{
        .proxyA = c.b2MakeProxy(&vas, vas.len, 0.0),
        .proxyB = c.b2MakeProxy(&vbs, vbs.len, 0.0),
        .sweepA = b2.sweepV(
            c.b2Vec2_zero,
            c.b2Vec2_zero,
            c.b2Vec2_zero,
            c.b2Rot_identity,
            c.b2Rot_identity,
        ),
        .sweepB = b2.sweepV(
            c.b2Vec2_zero,
            c.b2Vec2_zero,
            b2.vec2(-2, 0),
            c.b2Rot_identity,
            c.b2Rot_identity,
        ),
        .tMax = 1.0,
    };
    const output = c.b2TimeOfImpact(&input);

    try std.testing.expect(output.state == c.b2_toiStateHit);
    try std.testing.expectApproxEqAbs(0, output.t - 0.5, c.b2_linearSlop);
}

test "math" {
    const zero = c.b2Vec2_zero;
    const one = b2.vec2(1, 1);
    const two = b2.vec2(2, 2);

    var v = c.b2Add(one, two);
    try std.testing.expect(v.x == 3.0 and v.y == 3.0);

    v = c.b2Sub(zero, two);
    try std.testing.expect(v.x == -2.0 and v.y == -2.0);

    v = c.b2Add(two, two);
    try std.testing.expect(v.x != 5.0 and v.y != 5.0);

    const xf1 = b2.transformV(b2.vec2(-2, 3.0), c.b2MakeRot(1.0));
    const xf2 = b2.transformV(b2.vec2(1, 0), c.b2MakeRot(-2.0));

    const xf = c.b2MulTransforms(xf2, xf1);

    v = c.b2TransformPoint(xf2, c.b2TransformPoint(xf1, two));
    const u = c.b2TransformPoint(xf, two);

    try std.testing.expectApproxEqAbs(0, u.x - v.x, 10.0 * epsilon);
    try std.testing.expectApproxEqAbs(0, u.y - v.y, 10.0 * epsilon);

    v = c.b2TransformPoint(xf1, two);
    v = c.b2InvTransformPoint(xf1, v);

    try std.testing.expectApproxEqAbs(0, v.x - two.x, 8.0 * epsilon);
    try std.testing.expectApproxEqAbs(0, v.y - two.y, 8.0 * epsilon);
}

const capsule: b2.Capsule = b2.capsule(-1, 0, 1, 0, 1);
var box: b2.Polygon = undefined;
const segment: b2.Segment = b2.segment(0, 1, 0, -1);

const n = 4;

// removing / adding callconv(.C) makes this function shit itself
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

pub inline fn b2Dot(arg_a: c.b2Vec2, arg_b: c.b2Vec2) f32 {
    return (arg_a.x * arg_b.x) + (arg_a.y * arg_b.y);
}

test "shape mass" {
    const circle: c.b2Circle = .{
        .point = .{ .x = 1, .y = 0 },
        .radius = 1,
    };

    {
        //const md = c.b2ComputeCircleMass(&circle, 1.0);
        const md = b2ComputeCircleMass(&circle, 1.0);
        try std.testing.expectApproxEqAbs(0, md.mass - c.b2_pi, epsilon);
        //std.debug.print("{}\n{}\n", .{ circle.point, md });
        try std.testing.expectEqual(1, md.center.x);
        try std.testing.expectEqual(0, md.center.y);
        try std.testing.expectApproxEqAbs(0, md.I - 1.5 * c.b2_pi, epsilon);
    }

    //{
    //    const radius = capsule.radius;
    //    const length = c.b2Distance(capsule.point1, capsule.point2);

    //    const md = c.b2ComputeCapsuleMass(&capsule, 1.0);

    //    // Box that full contains capsule
    //    const r = c.b2MakeBox(radius, radius + 0.5 * length);
    //    const mdr = c.b2ComputePolygonMass(&r, 1.0);

    //    // Approximate capsule using convex hull
    //    var points: [2 * n]b2.Vec2 = undefined;
    //    const d = c.b2_pi / (n - 1.0);
    //    var angle = -0.5 * c.b2_pi;
    //    for (0..n) |i| {
    //        points[i].x = 1.0 + radius * @cos(angle);
    //        points[i].y = radius * @sin(angle);
    //        angle += d;
    //    }

    //    angle = 0.5 * c.b2_pi;
    //    for (n..2 * n) |i| {
    //        points[i].x = -1.0 + radius * @cos(angle);
    //        points[i].y = radius * @sin(angle);
    //        angle += d;
    //    }

    //    const hull = c.b2ComputeHull(&points, 2 * n);
    //    const ac = c.b2MakePolygon(&hull, 0.0);
    //    const ma = c.b2ComputePolygonMass(&ac, 1.0);

    //    try std.testing.expect(ma.mass < md.mass and md.mass < mdr.mass);
    //    try std.testing.expect(ma.I < md.I and md.I < mdr.I);
    //}

    //{
    //    const md = c.b2ComputePolygonMass(&box, 1.0);
    //    try std.testing.expectApproxEqAbs(0, md.mass - 4.0, epsilon);
    //    try std.testing.expectApproxEqAbs(0, md.center.x, epsilon);
    //    try std.testing.expectApproxEqAbs(0, md.center.y, epsilon);
    //    try std.testing.expectApproxEqAbs(0, md.I - 8.0 / 3.0, 2.0 * epsilon);
    //}
}

//static int ShapeAABBTest(void)
//{
//    {
//        b2AABB b = b2ComputeCircleAABB(&circle, b2Transform_identity);
//        ENSURE_SMALL(b.lowerBound.x, FLT_EPSILON);
//        ENSURE_SMALL(b.lowerBound.y + 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(b.upperBound.x - 2.0f, FLT_EPSILON);
//        ENSURE_SMALL(b.upperBound.y - 1.0f, FLT_EPSILON);
//    }
//
//    {
//        b2AABB b = b2ComputePolygonAABB(&box, b2Transform_identity);
//        ENSURE_SMALL(b.lowerBound.x + 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(b.lowerBound.y + 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(b.upperBound.x - 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(b.upperBound.y - 1.0f, FLT_EPSILON);
//    }
//
//    {
//        b2AABB b = b2ComputeSegmentAABB(&segment, b2Transform_identity);
//        ENSURE_SMALL(b.lowerBound.x, FLT_EPSILON);
//        ENSURE_SMALL(b.lowerBound.y + 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(b.upperBound.x, FLT_EPSILON);
//        ENSURE_SMALL(b.upperBound.y - 1.0f, FLT_EPSILON);
//    }
//
//    return 0;
//}
//
//static int PointInShapeTest(void)
//{
//    b2Vec2 p1 = {0.5f, 0.5f};
//    b2Vec2 p2 = {4.0f, -4.0f};
//
//    {
//        bool hit;
//        hit = b2PointInCircle(p1, &circle);
//        ENSURE(hit == true);
//        hit = b2PointInCircle(p2, &circle);
//        ENSURE(hit == false);
//    }
//
//    {
//        bool hit;
//        hit = b2PointInPolygon(p1, &box);
//        ENSURE(hit == true);
//        hit = b2PointInPolygon(p2, &box);
//        ENSURE(hit == false);
//    }
//
//    return 0;
//}
//
//static int RayCastShapeTest(void)
//{
//    b2RayCastInput input = {{-4.0f, 0.0f}, {8.0f, 0.0f}, 1.0f};
//
//    {
//        b2CastOutput output = b2RayCastCircle(&input, &circle);
//        ENSURE(output.hit);
//        ENSURE_SMALL(output.normal.x + 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(output.normal.y, FLT_EPSILON);
//        ENSURE_SMALL(output.fraction - 0.5f, FLT_EPSILON);
//    }
//
//    {
//        b2CastOutput output = b2RayCastPolygon(&input, &box);
//        ENSURE(output.hit);
//        ENSURE_SMALL(output.normal.x + 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(output.normal.y, FLT_EPSILON);
//        ENSURE_SMALL(output.fraction - 3.0f / 8.0f, FLT_EPSILON);
//    }
//
//    {
//        b2CastOutput output = b2RayCastSegment(&input, &segment, true);
//        ENSURE(output.hit);
//        ENSURE_SMALL(output.normal.x + 1.0f, FLT_EPSILON);
//        ENSURE_SMALL(output.normal.y, FLT_EPSILON);
//        ENSURE_SMALL(output.fraction - 0.5f, FLT_EPSILON);
//    }
//
//    return 0;
//}
