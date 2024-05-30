const std = @import("std");

const Cpu = @import("cpu.zig").Cpu;

pub fn main() !void {}

test "reading and writing works" {
    var cpu = Cpu{};

    // Writing //

    cpu.write_and_bump_pc([3]u8, [_]u8{ 0x11, 0x69, 0x42 });
    try std.testing.expect(std.mem.eql(u8, cpu.memory[0x0000..0x0003], &[_]u8{ 0x11, 0x69, 0x42 }));
    try std.testing.expect(cpu.pc == 0x0003);

    cpu.write(u16, 0xfffe, 0x4514);
    try std.testing.expect(std.mem.eql(u8, cpu.memory[0xfffe..], &[_]u8{ 0x14, 0x45 }));

    // Reading //

    try std.testing.expect(cpu.read(u16, 0x0001) == 0x4269);

    cpu.pc = 0xfffe;
    try std.testing.expect(cpu.read_and_bump_pc(u24) == 0x114514);
    try std.testing.expect(cpu.pc == 0x0001);
}
test "arithmetic" {
    var cpu = Cpu{};
    // ORA #69
    // ADC $00
    // STA $01
    cpu.write([6]u8, 0, [_]u8{ 0x09, 0x69, 0x65, 0x00, 0x85, 0x01 });

    std.debug.print("a=0x{x:0>2}, pc=0x{x:0>4}, mem={any}\n", .{ cpu.a, cpu.pc, cpu.memory[0..16] });

    cpu.step();
    std.debug.print("a=0x{x:0>2}, pc=0x{x:0>4}, mem={any}\n", .{ cpu.a, cpu.pc, cpu.memory[0..16] });

    cpu.step();
    std.debug.print("a=0x{x:0>2}, pc=0x{x:0>4}, mem={any}\n", .{ cpu.a, cpu.pc, cpu.memory[0..16] });

    cpu.step();
    std.debug.print("a=0x{x:0>2}, pc=0x{x:0>4}, mem={any}\n", .{ cpu.a, cpu.pc, cpu.memory[0..16] });
}
