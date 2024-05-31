const std = @import("std");
const Op = @import("op.zig").Op;

pub const Cpu = struct {
    a: u8 = 0,
    x: u8 = 0,
    y: u8 = 0,
    sp: u8 = 0xff,
    pc: u16 = 0x0000,
    flags: Flags = .{},
    memory: [64 * 1024]u8 = [_]u8{0} ** (64 * 1024),

    pub fn init() Cpu {
        var cpu = Cpu{};
        cpu.reset();
        return cpu;
    }

    pub fn step(self: *Cpu) void {
        if (Op.init(self)) |op| {
            op.run(self);
        }
    }

    pub fn reset(self: *Cpu) void {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.sp = 0xff;
        self.pc = self.read(u16, 0xfffc);
        self.flags.interrupt_disabled = true;
    }
    pub fn nmi(self: *Cpu) void {
        self.stackPush(u16, self.pc);
        self.saveFlags(false);
        // Jump to interrupt vector
        self.pc = self.read(u16, 0xfffa);
    }
    pub fn irq(self: *Cpu) void {
        self.stackPush(u16, self.pc);
        self.saveFlags(false);
        // Jump to interrupt vector
        self.pc = self.read(u16, 0xfffe);
    }

    pub fn debug(self: *const Cpu) void {
        std.debug.print("a=0x{x:0>2}, x=0x{x:0>2}, y=0x{x:0>2}, pc=0x{x:0>4}, sp=0x01{x:0>2} [{}]\n", .{ self.a, self.x, self.y, self.pc, self.sp, self.flags });
    }

    pub fn read(self: *const Cpu, comptime T: type, addr: u16) T {
        var _ptr = addr;
        return self.readAndBump(T, &_ptr);
    }
    pub fn readAndBumpPc(self: *Cpu, comptime T: type) T {
        return self.readAndBump(T, &self.pc);
    }

    pub fn write(self: *Cpu, comptime T: type, addr: u16, val: T) void {
        var _ptr = addr;
        return self.writeAndBump(T, &_ptr, val);
    }
    pub fn writeAndBumpPc(self: *Cpu, comptime T: type, val: T) void {
        return self.writeAndBump(T, &self.pc, val);
    }

    pub inline fn setA(self: *Cpu, new: u8) void {
        self.a = new;
        self.updateNZFlags(new);
    }
    pub inline fn setX(self: *Cpu, new: u8) void {
        self.x = new;
        self.updateNZFlags(new);
    }
    pub inline fn setY(self: *Cpu, new: u8) void {
        self.y = new;
        self.updateNZFlags(new);
    }
    pub inline fn updateNZFlags(self: *Cpu, new: u8) void {
        self.flags.negative = new & 0x80 != 0;
        self.flags.zero = new == 0;
    }

    pub fn saveFlags(self: *Cpu, is_break: bool) void {
        var flags: u8 = @bitCast(self.flags);
        if (is_break) flags |= 0b00100000; // Set break and reserved bits
        self.stackPush(u8, flags);
    }
    pub fn restoreFlags(self: *Cpu) void {
        const mask: u8 = 0b11001111; // Bits we care about (everything except break and reserved)
        var flags: u8 = @bitCast(self.flags);
        flags &= ~mask; // Clear the important bits
        flags |= (self.stackPop(u8) & mask); // Add them back
        self.flags = @bitCast(flags);
    }

    pub fn stackPush(self: *Cpu, comptime T: type, val: T) void {
        const info = readWriteInfo(T);
        const basis: u16 = 0x0100;
        self.sp -%= info.bytes;

        var ptr = self.sp;

        for (serialize(T, val)) |v| {
            ptr +%= 1;
            self.memory[basis + ptr] = v;
        }
    }
    pub fn stackPop(self: *Cpu, comptime T: type) T {
        const info = readWriteInfo(T);
        const basis: u16 = 0x0100;

        var buf: [info.bytes]u8 = undefined;
        for (&buf) |*m| {
            self.sp +%= 1;
            m.* = self.memory[basis + self.sp];
        }
        return deserialize(T, buf);
    }

    pub inline fn branch(self: *Cpu, criterion: bool, offset: i8) void {
        if (!criterion) return;
        if (offset < 0) {
            self.pc -%= @abs(offset);
        } else {
            self.pc +%= @abs(offset);
        }
    }

    fn readAndBump(self: *const Cpu, comptime T: type, ptr: *u16) T {
        const info = readWriteInfo(T);

        var buf: [info.bytes]u8 = undefined;
        for (&buf) |*m| {
            m.* = self.memory[ptr.*];
            ptr.* +%= 1;
        }

        return deserialize(T, buf);
    }
    fn writeAndBump(self: *Cpu, comptime T: type, ptr: *u16, val: T) void {
        for (serialize(T, val)) |v| {
            self.memory[ptr.*] = v;
            ptr.* +%= 1;
        }
    }
    fn readWriteInfo(comptime T: type) struct { int_size: comptime_int, bytes: comptime_int } {
        switch (@typeInfo(T)) {
            .Int => |i| {
                const int_size = @divExact(i.bits, 8);
                return .{ .int_size = int_size, .bytes = int_size };
            },
            .Array => |a| switch (@typeInfo(a.child)) {
                .Int => |i| {
                    const int_size = @divExact(i.bits, 8);
                    return .{ .int_size = int_size, .bytes = a.len * int_size };
                },
                else => {},
            },
            else => {},
        }
        @compileError("T can only be an integer or an array of integers");
    }
    fn deserialize(comptime T: type, buf: [readWriteInfo(T).bytes]u8) T {
        const info = readWriteInfo(T);

        switch (@typeInfo(T)) {
            .Int => return std.mem.readInt(T, &buf, .little),
            .Array => |a| {
                var ret: [a.len]a.child = undefined;
                for (0..a.len) |idx| {
                    ret[idx] = std.mem.readInt(a.child, buf[idx * info.int_size ..][0..info.int_size], .little);
                }
                return ret;
            },
            else => comptime unreachable,
        }
    }
    fn serialize(comptime T: type, val: T) [readWriteInfo(T).bytes]u8 {
        const info = readWriteInfo(T);
        var buf: [info.bytes]u8 = undefined;

        switch (@typeInfo(T)) {
            .Int => std.mem.writeInt(T, &buf, val, .little),
            .Array => |a| for (val, 0..) |v, idx| {
                std.mem.writeInt(a.child, buf[idx * info.int_size ..][0..info.int_size], v, .little);
            },
            else => comptime unreachable,
        }
        return buf;
    }
};

const Flags = packed struct(u8) {
    carry: bool = false,
    zero: bool = false,
    interrupt_disabled: bool = false,
    decimal_mode: bool = false,
    _break: bool = false, // not real
    _reserved: bool = true, // not real
    overflow: bool = false,
    negative: bool = false,

    pub fn format(value: Flags, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;

        const mask: u8 = 0b10000000;
        const val: u8 = @bitCast(value);

        var s = "nv_bdizc".*;
        for (&s, 0..) |*b, idx| {
            if (val & (mask >> @truncate(idx)) != 0) {
                b.* = std.ascii.toUpper(b.*);
            }
        }

        try writer.writeAll(&s);
    }
};

test "reading and writing works" {
    var cpu = Cpu.init();

    // Writing //
    cpu.writeAndBumpPc([2]u16, [_]u16{ 0x6911, 0xff42 });
    try std.testing.expectEqualSlices(u8, cpu.memory[0x0000..0x0003], &[_]u8{ 0x11, 0x69, 0x42 });
    try std.testing.expectEqual(cpu.pc, 0x0004);

    cpu.write(u16, 0xfffe, 0x4514);
    try std.testing.expectEqualSlices(u8, cpu.memory[0xfffe..], &[_]u8{ 0x14, 0x45 });

    // Reading //
    try std.testing.expectEqualSlices(u16, &cpu.read([1]u16, 0x0001), &[_]u16{0x4269});

    cpu.pc = 0xfffe;
    try std.testing.expectEqual(cpu.readAndBumpPc(u24), 0x114514);
    try std.testing.expectEqual(cpu.pc, 0x0001);
}

test "arithmetic" {
    var cpu = Cpu.init();

    // Write interrupt vector
    cpu.write(u16, 0xfffc, 0x2000);

    // ORA #79
    // ADC $2000
    // STA $2001
    cpu.write([8]u8, 0x2000, [_]u8{ 0x09, 0x79, 0x6D, 0x00, 0x20, 0x8D, 0x01, 0x20 });

    cpu.reset();
    cpu.debug();

    for (0..3) |_| {
        cpu.step();
        cpu.debug();
    }

    try std.testing.expect(cpu.flags.negative);
    try std.testing.expectEqual(0x82, cpu.a);
    try std.testing.expectEqual(0x82, cpu.memory[0x2001]);
}

test "stack operations" {
    var cpu = Cpu.init();

    // Pushing
    cpu.stackPush(u8, 0x69);
    try std.testing.expectEqual(0x69, cpu.memory[0x01ff]);
    try std.testing.expectEqual(0xfe, cpu.sp);

    cpu.stackPush(u16, 0x0420);
    try std.testing.expectEqualSlices(u8, &[_]u8{ 0x20, 0x04 }, cpu.memory[0x01fd..][0..2]);
    try std.testing.expectEqual(0xfc, cpu.sp);

    cpu.stackPush([5]u8, "hello".*);
    try std.testing.expectEqualStrings("hello", cpu.memory[0x01f8..][0..5]);
    try std.testing.expectEqual(0xf7, cpu.sp);

    // Popping
    try std.testing.expectEqualStrings("hello", &cpu.stackPop([5]u8));
    try std.testing.expectEqual(0xfc, cpu.sp);

    try std.testing.expectEqual(0x20, cpu.stackPop(u8));
    try std.testing.expectEqual(0xfd, cpu.sp);

    try std.testing.expectEqual(0x6904, cpu.stackPop(u16));
    try std.testing.expectEqual(0xff, cpu.sp);
}
