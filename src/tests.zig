const std = @import("std");
const b2 = @import("init.zig");
const c = b2.c;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectApproxEqAbs = std.testing.expectApproxEqAbs;

const epsilon = @sqrt(std.math.floatEps(f32));

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
test "hello world" {
    // Define the gravity vector.
    const gravity = b2.vec2(0.0, -10.0);

    // Construct a world object, which will hold and simulate the rigid bodies.
    var worldDef = c.b2DefaultWorldDef();
    worldDef.gravity = gravity;

    const worldId = c.b2CreateWorld(&worldDef);
    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned ids, so be careful about your world management.
    defer c.b2DestroyWorld(worldId);

    // Define the ground body.
    var groundBodyDef = c.b2DefaultBodyDef();
    groundBodyDef.position = b2.vec2(0.0, -10.0);

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    const groundBodyId = c.b2CreateBody(worldId, &groundBodyDef);

    // Define the ground box shape. The extents are the half-widths of the box.
    const groundBox = b2.makeBox(50.0, 10.0);

    // Add the box shape to the ground body.
    const groundShapeDef = c.b2DefaultShapeDef();
    _ = c.b2CreatePolygonShape(groundBodyId, &groundShapeDef, &groundBox);

    // Define the dynamic body. We set its position and call the body factory.
    var bodyDef = c.b2DefaultBodyDef();
    bodyDef.type = c.b2_dynamicBody;
    bodyDef.position = b2.vec2(0.0, 4.0);

    const bodyId = c.b2CreateBody(worldId, &bodyDef);

    // Define another box shape for our dynamic body.
    const dynamicBox = c.b2MakeBox(1.0, 1.0);

    // Define the dynamic body shape
    var shapeDef = c.b2DefaultShapeDef();

    // Set the box density to be non-zero, so it will be dynamic.
    shapeDef.density = 1.0;

    // Override the default friction.
    shapeDef.friction = 0.3;

    // Add the shape to the body.
    _ = c.b2CreatePolygonShape(bodyId, &shapeDef, &dynamicBox);

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 4 sub-steps. This provides a high quality simulation
    // in most game scenarios.
    const timeStep: f32 = 1.0 / 60.0;
    const subStepCount = 4;

    var position = c.b2Body_GetPosition(bodyId);
    var angle = c.b2Body_GetRotation(bodyId);

    // This is our little game loop.
    for (0..90) |_| {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        c.b2World_Step(worldId, timeStep, subStepCount);

        // Now print the position and angle of the body.
        position = c.b2Body_GetPosition(bodyId);
        angle = c.b2Body_GetRotation(bodyId);

        //printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
    }

    try expect(@abs(position.x) < 0.01);
    try expect(@abs(position.y - 1.00) < 0.01);
    try expect(@abs(c.b2Rot_GetAngle(angle)) < 0.01);
}

test "empty world" {
    const worldDef = c.b2DefaultWorldDef();
    const worldId = c.b2CreateWorld(&worldDef);
    try expect(c.b2World_IsValid(worldId) == true);

    const timeStep = 1.0 / 60.0;
    const subStepCount = 1;

    for (0..60) |_| {
        c.b2World_Step(worldId, timeStep, subStepCount);
    }

    c.b2DestroyWorld(worldId);

    try expect(c.b2World_IsValid(worldId) == false);
}

test "destroy all bodies world" {
    const body_count = 10;

    const worldDef = c.b2DefaultWorldDef();
    const worldId = c.b2CreateWorld(&worldDef);
    try expect(c.b2World_IsValid(worldId) == true);

    var count: u32 = 0;
    var creating = true;

    var bodyIds: [body_count]b2.BodyId = undefined;
    var bodyDef = c.b2DefaultBodyDef();
    bodyDef.type = c.b2_dynamicBody;
    const square = b2.makeSquare(0.5);

    for (0..2 * body_count + 10) |_| {
        if (creating) {
            if (count < body_count) {
                bodyIds[count] = c.b2CreateBody(worldId, &bodyDef);

                const shapeDef = c.b2DefaultShapeDef();
                _ = c.b2CreatePolygonShape(bodyIds[count], &shapeDef, &square);
                count += 1;
            } else {
                creating = false;
            }
        } else if (count > 0) {
            c.b2DestroyBody(bodyIds[count - 1]);
            bodyIds[count - 1] = c.b2_nullBodyId;
            count -= 1;
        }

        c.b2World_Step(worldId, 1.0 / 60.0, 3);
    }

    const counters = c.b2World_GetCounters(worldId);
    try expect(counters.bodyCount == 0);

    c.b2DestroyWorld(worldId);

    try expect(c.b2World_IsValid(worldId) == false);
}

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
        try expectEqual(value, values[i]);
    }
}

test "collision" {
    var a = b2.aabbS(-1, -1, -2, -2);
    try expect(c.b2IsValidAABB(a) == false);

    a.upperBound = b2.vec2(1, 1);
    try expect(c.b2IsValidAABB(a) == true);

    const b = b2.aabbS(2, 2, 4, 4);
    try expect(c.b2AABB_Overlaps(a, b) == false);
    try expect(c.b2AABB_Contains(a, b) == false);

    const p1 = b2.vec2(-2, 0);
    const p2 = b2.vec2(2, 0);

    const output = c.b2AABB_RayCast(a, p1, p2);
    try expect(output.hit == true);
    try expect(0.1 < output.fraction and output.fraction < 0.9);
}

test "segment distance" {
    const p1: b2.Vec2 = b2.vec2(-1, -1);
    const q1: b2.Vec2 = b2.vec2(-1, 1);
    const p2: b2.Vec2 = b2.vec2(2, 0);
    const q2: b2.Vec2 = b2.vec2(1, 0);

    const result = c.b2SegmentDistance(p1, q1, p2, q2);

    try expectApproxEqAbs(0, result.fraction1 - 0.5, epsilon);
    try expectApproxEqAbs(0, result.fraction2 - 1.0, epsilon);
    try expectApproxEqAbs(0, result.closest1.x + 1.0, epsilon);
    try expectApproxEqAbs(0, result.closest1.y, epsilon);
    try expectApproxEqAbs(0, result.closest2.x - 1.0, epsilon);
    try expectApproxEqAbs(0, result.closest2.y, epsilon);
    try expectApproxEqAbs(0, result.distanceSquared - 4.0, epsilon);
}

test "shape distance" {
    var vas = [_]b2.Vec2{
        b2.vec2(-1, -1),
        b2.vec2(1, -1),
        b2.vec2(1, 1),
        b2.vec2(-1, 1),
    };
    var vbs = [_]b2.Vec2{
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

    var cache: c.b2SimplexCache = .{};
    const output = c.b2ShapeDistance(&cache, &input, null, 0);

    try expectApproxEqAbs(0, output.distance - 1.0, epsilon);
}

test "shape cast" {
    var vas = [_]b2.Vec2{
        b2.vec2(-1, -1),
        b2.vec2(1, -1),
        b2.vec2(1, 1),
        b2.vec2(-1, 1),
    };
    var vbs = [_]b2.Vec2{
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

    try expect(output.hit);
    try expectApproxEqAbs(0, output.fraction - 0.5, c.B2_LINEAR_SLOP());
}

test "time of impact" {
    var vas = [_]b2.Vec2{
        b2.vec2(-1, -1),
        b2.vec2(1, -1),
        b2.vec2(1, 1),
        b2.vec2(-1, 1),
    };
    var vbs = [_]b2.Vec2{
        b2.vec2(2, -1),
        b2.vec2(2, 1),
    };

    var input: c.b2TOIInput = .{
        .proxyA = c.b2MakeProxy(&vas, vas.len, 0.0),
        .proxyB = c.b2MakeProxy(&vbs, vbs.len, 0.0),
        .sweepA = b2.sweep(
            .{},
            .{},
            .{},
            c.b2Rot_identity,
            c.b2Rot_identity,
        ),
        .sweepB = b2.sweep(
            .{},
            .{},
            b2.vec2(-2, 0),
            c.b2Rot_identity,
            c.b2Rot_identity,
        ),
        .maxFraction = 1.0,
    };
    const output = c.b2TimeOfImpact(&input);

    try expectEqual(@as(c_uint, @intCast(c.b2_toiStateHit)), output.state);
    try expectApproxEqAbs(0, output.fraction - 0.5, c.B2_LINEAR_SLOP());
}

test "math" {
    const zero = b2.Vec2{};
    const one = b2.vec2(1, 1);
    const two = b2.vec2(2, 2);

    var v = c.b2Add(one, two);
    try expectEqual(b2.vec2(3, 3), v);

    v = c.b2Sub(zero, two);
    try expectEqual(b2.vec2(-2, -2), v);

    v = c.b2Add(two, two);
    try expect(v.x != 5.0 and v.y != 5.0);

    const xf1 = b2.transform(b2.vec2(-2, 3.0), c.b2MakeRot(1.0));
    const xf2 = b2.transform(b2.vec2(1, 0), c.b2MakeRot(-2.0));

    const xf = c.b2MulTransforms(xf2, xf1);

    v = c.b2TransformPoint(xf2, c.b2TransformPoint(xf1, two));
    const u = c.b2TransformPoint(xf, two);

    try expectApproxEqAbs(0, u.x - v.x, 10.0 * epsilon);
    try expectApproxEqAbs(0, u.y - v.y, 10.0 * epsilon);

    v = c.b2TransformPoint(xf1, two);
    v = c.b2InvTransformPoint(xf1, v);

    try expectApproxEqAbs(0, v.x - two.x, 8.0 * epsilon);
    try expectApproxEqAbs(0, v.y - two.y, 8.0 * epsilon);
}

const capsule: b2.Capsule = b2.capsuleS(-1, 0, 1, 0, 1);
const circle = b2.circleS(1, 0, 1);
const box: b2.Polygon = b2.makeBox(1, 1);
const segment: b2.Segment = b2.segmentS(0, 1, 0, -1);

test "shape mass" {
    const n = 4;

    {
        const md = c.b2ComputeCircleMass(&circle, 1.0);
        try expectApproxEqAbs(0, md.mass - c.B2_PI, epsilon);
        try expectEqual(1, md.center.x);
        try expectEqual(0, md.center.y);
        try expectApproxEqAbs(0, md.rotationalInertia - 1.5 * c.B2_PI, epsilon);
    }

    {
        const radius = capsule.radius;
        const length = c.b2Distance(capsule.center1, capsule.center2);

        const md = c.b2ComputeCapsuleMass(&capsule, 1.0);

        // Box that full contains capsule
        const r = c.b2MakeBox(radius, radius + 0.5 * length);
        const mdr = c.b2ComputePolygonMass(&r, 1.0);

        // Approximate capsule using convex hull
        var points: [2 * n]b2.Vec2 = undefined;
        const d = c.B2_PI / (n - 1.0);
        var angle = -0.5 * c.B2_PI;
        for (0..n) |i| {
            points[i].x = 1.0 + radius * @cos(angle);
            points[i].y = radius * @sin(angle);
            angle += d;
        }

        angle = 0.5 * c.B2_PI;
        for (n..2 * n) |i| {
            points[i].x = -1.0 + radius * @cos(angle);
            points[i].y = radius * @sin(angle);
            angle += d;
        }

        const hull = c.b2ComputeHull(&points, 2 * n);
        const ac = c.b2MakePolygon(&hull, 0.0);
        const ma = c.b2ComputePolygonMass(&ac, 1.0);

        try expect(ma.mass < md.mass and md.mass < mdr.mass);
        try expect(ma.rotationalInertia < md.rotationalInertia and md.rotationalInertia < mdr.rotationalInertia);
    }

    {
        const md = c.b2ComputePolygonMass(&box, 1.0);
        try expectApproxEqAbs(0, md.mass - 4.0, epsilon);
        try expectApproxEqAbs(0, md.center.x, epsilon);
        try expectApproxEqAbs(0, md.center.y, epsilon);
        try expectApproxEqAbs(0, md.rotationalInertia - 8.0 / 3.0, 2.0 * epsilon);
    }
}

test "shape AABB" {
    {
        const b = c.b2ComputeCircleAABB(&circle, c.b2Transform_identity);
        try expectApproxEqAbs(0, b.lowerBound.x, epsilon);
        try expectApproxEqAbs(0, b.lowerBound.y + 1.0, epsilon);
        try expectApproxEqAbs(0, b.upperBound.x - 2.0, epsilon);
        try expectApproxEqAbs(0, b.upperBound.y - 1.0, epsilon);
    }

    {
        const b = c.b2ComputePolygonAABB(&box, c.b2Transform_identity);
        try expectApproxEqAbs(0, b.lowerBound.x + 1.0, epsilon);
        try expectApproxEqAbs(0, b.lowerBound.y + 1.0, epsilon);
        try expectApproxEqAbs(0, b.upperBound.x - 1.0, epsilon);
        try expectApproxEqAbs(0, b.upperBound.y - 1.0, epsilon);
    }

    {
        const b = c.b2ComputeSegmentAABB(&segment, c.b2Transform_identity);
        try expectApproxEqAbs(0, b.lowerBound.x, epsilon);
        try expectApproxEqAbs(0, b.lowerBound.y + 1.0, epsilon);
        try expectApproxEqAbs(0, b.upperBound.x, epsilon);
        try expectApproxEqAbs(0, b.upperBound.y - 1.0, epsilon);
    }
}

test "point in shape" {
    const p1 = b2.vec2(0.5, 0.5);
    const p2 = b2.vec2(4.0, -4.0);

    {
        var hit: bool = undefined;
        hit = c.b2PointInCircle(p1, &circle);
        try expect(hit == true);
        hit = c.b2PointInCircle(p2, &circle);
        try expect(hit == false);
    }

    {
        var hit: bool = undefined;
        hit = c.b2PointInPolygon(p1, &box);
        try expect(hit == true);
        hit = c.b2PointInPolygon(p2, &box);
        try expect(hit == false);
    }
}

test "raycast shape" {
    const input = b2.rayCastInput(b2.vec2(-4.0, 0.0), b2.vec2(8.0, 0.0), 1.0);

    {
        const output = c.b2RayCastCircle(&input, &circle);
        try expect(output.hit);
        try expectApproxEqAbs(0, output.normal.x + 1.0, epsilon);
        try expectApproxEqAbs(0, output.normal.y, epsilon);
        try expectApproxEqAbs(0, output.fraction - 0.5, epsilon);
    }

    {
        const output = c.b2RayCastPolygon(&input, &box);
        try expect(output.hit);
        try expectApproxEqAbs(0, output.normal.x + 1.0, epsilon);
        try expectApproxEqAbs(0, output.normal.y, epsilon);
        try expectApproxEqAbs(0, output.fraction - 3.0 / 8.0, epsilon);
    }

    {
        const output = c.b2RayCastSegment(&input, &segment, true);
        try expect(output.hit);
        try expectApproxEqAbs(0, output.normal.x + 1.0, epsilon);
        try expectApproxEqAbs(0, output.normal.y, epsilon);
        try expectApproxEqAbs(0, output.fraction - 0.5, epsilon);
    }
}

test "table" {
    const set_span = 317;
    const item_count = ((set_span * set_span - set_span) / 2);

    const n: i32 = set_span;
    const itemCount: u32 = item_count;
    var removed: [item_count]bool = .{false} ** item_count;

    for (0..1) |_| {
        var set = c.b2CreateSet(16);
        defer c.b2DestroySet(&set);

        // Fill set
        for (0..n) |i| {
            for (i + 1..n) |j| {
                const key = c.B2_SHAPE_PAIR_KEY(i, j);
                _ = c.b2AddKey(&set, key);
            }
        }

        try expectEqual(itemCount, set.count);

        // Remove a portion of the set
        var k: u32 = 0;
        var removeCount: u32 = 0;
        for (0..n) |i| {
            for (i + 1..n) |j| {
                if (j == i + 1) {
                    const key = c.B2_SHAPE_PAIR_KEY(i, j);
                    _ = c.b2RemoveKey(&set, key);
                    removed[k] = true;
                    k += 1;
                    removeCount += 1;
                } else {
                    removed[k] = false;
                    k += 1;
                }
            }
        }

        try expectEqual((itemCount - removeCount), set.count);

        // Test key search
        // ~5ns per search on an AMD 7950x
        // const timer = c.b2CreateTimer();

        k = 0;
        for (0..n) |i| {
            for (i + 1..n) |j| {
                const key = c.B2_SHAPE_PAIR_KEY(j, i);
                try expect(c.b2ContainsKey(&set, key) or removed[k]);
                k += 1;
            }
        }

        // uint64_t ticks = b2GetTicks(&timer);
        // printf("set ticks = %llu\n", ticks);

        // const ms = c.b2GetMilliseconds(&timer);
        // printf("set: count = %d, b2ContainsKey = %.5f ms, ave = %.5f us\n", itemCount, ms, 1000.0f * ms / itemCount);

        // Remove all keys from set
        for (0..n) |i| {
            for (i + 1..n) |j| {
                const key = c.B2_SHAPE_PAIR_KEY(i, j);
                _ = c.b2RemoveKey(&set, key);
            }
        }

        try expect(set.count == 0);
    }
}
