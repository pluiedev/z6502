const std = @import("std");
pub const cpu = @import("cpu.zig");
pub const op = @import("op.zig");

pub fn main() !void {}

test {
    std.testing.refAllDeclsRecursive(@This());
}
