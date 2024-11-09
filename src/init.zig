const std = @import("std");
const assert = std.debug.assert;

pub const c = @cImport({
    // api includes
    @cInclude("box2d/api.h");
    @cInclude("box2d/box2d.h");
    @cInclude("box2d/aabb.h");
    @cInclude("box2d/callbacks.h");
    @cInclude("box2d/color.h");
    @cInclude("box2d/constants.h");
    @cInclude("box2d/debug_draw.h");
    @cInclude("box2d/distance.h");
    @cInclude("box2d/dynamic_tree.h");
    @cInclude("box2d/event_types.h");
    @cInclude("box2d/geometry.h");
    @cInclude("box2d/hull.h");
    @cInclude("box2d/id.h");
    @cInclude("box2d/joint_types.h");
    @cInclude("box2d/manifold.h");
    @cInclude("box2d/math.h");
    @cInclude("box2d/math_types.h");
    @cInclude("box2d/timer.h");
    @cInclude("box2d/types.h");

    // src includes
    @cInclude("box2d/allocate.h");
    @cInclude("box2d/array.h");
    @cInclude("box2d/bitset.h");
    @cInclude("box2d/broad_phase.h");
    @cInclude("box2d/body.h");
    @cInclude("box2d/block_allocator.h");
    @cInclude("box2d/core.h");
    @cInclude("box2d/contact_solver.h");
    @cInclude("box2d/contact.h");
    @cInclude("box2d/constraint_graph.h");
    @cInclude("box2d/island.h");
    @cInclude("box2d/joint.h");
    @cInclude("box2d/table.h");
    @cInclude("box2d/world.h");
    @cInclude("box2d/stack_allocator.h");
    @cInclude("box2d/allocate.h");
    @cInclude("box2d/solver.h");
    @cInclude("box2d/pool.h");
    @cInclude("box2d/polygon_shape.h");
});

pub const WorldId = c.b2WorldId;
pub const BodyId = c.b2BodyId;
pub const Vec2 = c.b2Vec2;
pub const Rot = c.b2Rot;
pub const AABB = c.b2AABB;
pub const Circle = c.b2Circle;
pub const Capsule = c.b2Capsule;
pub const Transform = c.b2Transform;
pub const Polygon = c.b2Polygon;
pub const Segment = c.b2Segment;
pub const RayCastInput = c.b2RayCastInput;
pub const Sweep = c.b2Sweep;
pub const ShapeId = c.b2ShapeId;

pub inline fn isValid(a: f32) bool {
    return !std.math.isNan(a) and !std.math.isInf(a);
}

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

pub fn makeBox(
    hx: f32,
    hy: f32,
) Polygon {
    assert(isValid(hx) and hx > 0.0);
    assert(isValid(hy) and hy > 0.0);

    return .{
        .count = 4,
        .vertices = .{
            vec2(-hx, -hy),
            vec2(hx, -hy),
            vec2(hx, hy),
            vec2(-hx, hy),
        } ++ .{undefined} ** 4,
        .normals = .{
            vec2(0.0, -1.0),
            vec2(1.0, 0.0),
            vec2(0.0, 1.0),
            vec2(-1.0, 0.0),
        } ++ .{undefined} ** 4,
        .radius = 0.0,
        .centroid = .{},
    };
}

pub inline fn makeSquare(len: f32) Polygon {
    return makeBox(len, len);
}

pub inline fn aabbS(
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

pub inline fn aabb(low: Vec2, upper: Vec2) AABB {
    return .{
        .lowerBound = low,
        .upperBound = upper,
    };
}

pub inline fn circleS(
    point_x: anytype, // castable to float
    point_y: anytype, // castable to float
    radius: anytype, // castable to float
) Circle {
    return .{ .point = vec2(point_x, point_y), .radius = floatFromAny(f32, radius) };
}

pub inline fn circle(
    point: Vec2,
    radius: anytype, // castable to float
) Circle {
    return .{ .point = point, .radius = floatFromAny(f32, radius) };
}

pub inline fn capsuleS(
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

pub inline fn capsule(
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

pub inline fn segmentS(
    point1_x: anytype, // castable to float
    point1_y: anytype, // castable to float
    point2_x: anytype, // castable to float
    point2_y: anytype, // castable to float
) Segment {
    return .{ .point1 = vec2(point1_x, point1_y), .point2 = vec2(point2_x, point2_y) };
}

pub inline fn segment(
    point1: Vec2,
    point2: Vec2,
) Segment {
    return .{ .point1 = point1, .point2 = point2 };
}

pub inline fn polygon(
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

pub inline fn transformS(
    p_x: anytype, // castable to float
    p_y: anytype, // castable to float
    q_s: anytype, // castable to float
    q_c: anytype, // castable to float
) Transform {
    return .{ .p = vec2(p_x, p_y), .q = rot(q_s, q_c) };
}

pub inline fn transform(p: Vec2, q: Rot) Transform {
    return .{ .p = p, .q = q };
}

pub inline fn sweep(
    local_center: Vec2,
    c1: Vec2,
    c2: Vec2,
    q1: Rot,
    q2: Rot,
) Sweep {
    return .{
        .localCenter = local_center,
        .c1 = c1,
        .c2 = c2,
        .q1 = q1,
        .q2 = q2,
    };
}

pub inline fn rayCastInput(
    origin: Vec2,
    translation: Vec2,
    max_fraction: anytype, // castable to float
) RayCastInput {
    return .{
        .origin = origin,
        .translation = translation,
        .maxFraction = max_fraction,
    };
}

inline fn floatFromAny(comptime T: type, v: anytype) T {
    return switch (@typeInfo(@TypeOf(v))) {
        .int => @floatFromInt(v),
        .float => @floatCast(v),
        .comptime_int, .comptime_float => v,
        else => @compileError("Cannot convert " ++ @typeName(@TypeOf(v)) ++ " to float."),
    };
}

pub const BodyType = enum(c.b2BodyType) {
    static = 0,
    kinematic = 1,
    dynamic = 2,

    pub inline fn val(self: BodyType) c.b2BodyType {
        return @intFromEnum(self);
    }
};

pub const default = struct {
    pub const world_def: c.b2WorldDef = .{
        .gravity = .{ .x = 0, .y = -10 },
        .restitutionThreshold = 1.0 * c.b2_lengthUnitsPerMeter,
        .contactPushoutVelocity = 3.0 * c.b2_lengthUnitsPerMeter,
        .contactHertz = 30.0,
        .contactDampingRatio = 10.0,
        .jointHertz = 60.0,
        .jointDampingRatio = 2.0,
        .enableSleep = true,
        .enableContinous = true,
        .stackAllocatorCapacity = 1024 * 1024,
    };

    pub const body_def: c.b2BodyDef = .{
        .type = c.b2_staticBody,
        .gravityScale = 1.0,
        .enableSleep = true,
        .isAwake = true,
        .isEnabled = true,
    };

    pub const filter: c.b2Filter = .{
        .categoryBits = 0x00000001,
        .maskBits = 0xFFFFFFFF,
        .groupIndex = 0,
    };

    pub const query_filter: c.b2QueryFilter = .{
        .categoryBits = 0x00000001,
        .maskBits = 0xFFFFFFFF,
    };

    pub const shape_def: c.b2ShapeDef = .{
        .friction = 0.6,
        .density = 1.0,
        .filter = filter,
        .enableSensorEvents = true,
        .enableContactEvents = true,
    };

    pub const chain_def: c.b2ChainDef = .{
        .friction = 0.6,
        .filter = filter,
    };
};
