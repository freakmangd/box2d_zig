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

pub const Vec2 = c.b2Vec2;
pub const Rot = c.b2Rot;
pub const AABB = c.b2AABB;
pub const Circle = c.b2Circle;
pub const Capsule = c.b2Capsule;
pub const Transform = c.b2Transform;
pub const Polygon = c.b2Polygon;
pub const Segment = c.b2Segment;

pub inline fn vec2(
    x: anytype, // castable to float
    y: anytype, // castable to float
) Vec2 {
    return .{ .x = floatFromAny(f32, x), .y = floatFromAny(f32, y) };
}

pub inline fn rot(
    s: anytype, // castable to float
    _c: anytype, // castable to float
) Rot {
    return .{ .s = floatFromAny(f32, s), .c = floatFromAny(f32, _c) };
}

pub inline fn aabb(
    low_x: anytype, // castable to float
    low_y: anytype, // castable to float
    upper_x: anytype, // castable to float
    upper_y: anytype, // castable to float
) AABB {
    return .{
        .lowerBound = vec2(low_x, low_y),
        .upperBound = vec2(upper_x, upper_y),
    };
}

pub inline fn aabbV(low: c.b2Vec2, upper: c.b2Vec2) c.b2AABB {
    return .{
        .lowerBound = low,
        .upperBound = upper,
    };
}

pub inline fn circle(
    point_x: anytype, // castable to float
    point_y: anytype, // castable to float
    radius: anytype, // castable to float
) Circle {
    return .{ .point = vec2(point_x, point_y), .radius = floatFromAny(f32, radius) };
}

pub inline fn circleV(
    point: Vec2,
    radius: anytype, // castable to float
) Circle {
    return .{ .point = point, .radius = floatFromAny(f32, radius) };
}

pub inline fn capsule(
    point1_x: anytype, // castable to float
    point1_y: anytype, // castable to float
    point2_x: anytype, // castable to float
    point2_y: anytype, // castable to float
    radius: anytype, // castable to float
) Capsule {
    return .{
        .point1 = vec2(point1_x, point1_y),
        .point2 = vec2(point2_x, point2_y),
        .radius = floatFromAny(f32, radius),
    };
}

pub inline fn capsuleV(
    point1: Vec2,
    point2: Vec2,
    radius: anytype, // castable to float
) Capsule {
    return .{
        .point1 = point1,
        .point2 = point2,
        .radius = floatFromAny(f32, radius),
    };
}

pub inline fn segment(
    point1_x: anytype, // castable to float
    point1_y: anytype, // castable to float
    point2_x: anytype, // castable to float
    point2_y: anytype, // castable to float
) Segment {
    return .{ .point1 = vec2(point1_x, point1_y), .point2 = vec2(point2_x, point2_y) };
}

pub inline fn segmentV(
    point1: Vec2,
    point2: Vec2,
) Segment {
    return .{ .point1 = point1, .point2 = point2 };
}

pub inline fn polygonV(
    vertices: [8]Vec2,
    normals: [8]Vec2,
    centroid: Vec2,
    radius: anytype, // castable to float
    count: i32,
) Polygon {
    return .{
        .vertices = vertices,
        .normals = normals,
        .centroid = centroid,
        .radius = floatFromAny(f32, radius),
        .count = count,
    };
}

pub inline fn transform(
    p_x: anytype, // castable to float
    p_y: anytype, // castable to float
    q_s: anytype, // castable to float
    q_c: anytype, // castable to float
) Transform {
    return .{ .p = vec2(p_x, p_y), .q = rot(q_s, q_c) };
}

pub inline fn transformV(p: c.b2Vec2, q: c.b2Rot) Transform {
    return .{ .p = p, .q = q };
}

pub inline fn sweep(
    local_center_x: anytype, // castable to float
    local_center_y: anytype, // castable to float
    c1_x: anytype, // castable to float
    c1_y: anytype, // castable to float
    c2_x: anytype, // castable to float
    c2_y: anytype, // castable to float
    q1_s: anytype, // castable to float
    q1_c: anytype, // castable to float
    q2_s: anytype, // castable to float
    q2_c: anytype, // castable to float
) c.b2Sweep {
    return .{
        .localCenter = vec2(local_center_x, local_center_y),
        .c1 = vec2(c1_x, c1_y),
        .c2 = vec2(c2_x, c2_y),
        .q1 = rot(q1_s, q1_c),
        .q2 = rot(q2_s, q2_c),
    };
}

pub inline fn sweepV(
    local_center: c.b2Vec2,
    c1: c.b2Vec2,
    c2: c.b2Vec2,
    q1: c.b2Rot,
    q2: c.b2Rot,
) c.b2Sweep {
    return .{
        .localCenter = local_center,
        .c1 = c1,
        .c2 = c2,
        .q1 = q1,
        .q2 = q2,
    };
}

inline fn floatFromAny(comptime T: type, v: anytype) T {
    return switch (@typeInfo(@TypeOf(v))) {
        .Int => @floatFromInt(v),
        .Float => @floatCast(v),
        .ComptimeInt, .ComptimeFloat => v,
        else => @compileError("Cannot convert " ++ @typeName(@TypeOf(v)) ++ " to float."),
    };
}
