const std = @import("std");

const b2BitSet = extern struct {
    bits: [*]u64,
    blockCapacity: u32,
    blockCount: u32,
};

pub export fn b2SetBit(bitSet: *b2BitSet, bitIndex: u32) void {
    const blockIndex = bitIndex / 64;
    // TODO_ERIN support growing
    std.debug.assert(blockIndex < bitSet.blockCount);
    bitSet.bits[blockIndex] |= (@as(u64, 1) << @intCast(bitIndex % 64));
}
