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

    pub fn step(self: *Cpu) void {
        if (Op.init(self)) |op| {
            op.run(self);
        }
    }

    pub fn read_and_bump(self: *Cpu, comptime T: type, ptr: *u16) T {
        const num = switch (@typeInfo(T)) {
            .Int => |i| @divExact(i.bits, 8),
            .Array => |a| val: {
                std.debug.assert(a.child == u8);
                break :val a.len;
            },
            else => @compileError("T can only be an integer or an array of u8s"),
        };

        var buf: [num]u8 = undefined;
        for (&buf) |*m| {
            m.* = self.memory[ptr.*];
            ptr.* +%= 1;
        }
        return switch (@typeInfo(T)) {
            .Int => std.mem.readInt(T, &buf, .little),
            .Array => buf,
            else => @compileError("T can only be an integer or an array of u8s"),
        };
    }
    pub fn read(self: *Cpu, comptime T: type, addr: u16) T {
        var ptr = addr;
        return self.read_and_bump(T, &ptr);
    }
    pub fn read_and_bump_pc(self: *Cpu, comptime T: type) T {
        return self.read_and_bump(T, &self.pc);
    }

    pub fn write_and_bump(self: *Cpu, comptime T: type, ptr: *u16, val: T) void {
        const num = switch (@typeInfo(T)) {
            .Int => |i| @divExact(i.bits, 8),
            .Array => |a| val: {
                std.debug.assert(a.child == u8);
                break :val a.len;
            },
            else => @compileError("T can only be an integer or an array of u8s"),
        };
        var buf: [num]u8 = undefined;
        switch (@typeInfo(T)) {
            .Int => std.mem.writeInt(T, &buf, val, .little),
            .Array => buf = val,
            else => @compileError("T can only be an integer or an array of u8s"),
        }

        for (buf) |v| {
            self.memory[ptr.*] = v;
            ptr.* +%= 1;
        }
    }
    pub fn write(self: *Cpu, comptime T: type, addr: u16, val: T) void {
        var ptr = addr;
        return self.write_and_bump(T, &ptr, val);
    }
    pub fn write_and_bump_pc(self: *Cpu, comptime T: type, val: T) void {
        return self.write_and_bump(T, &self.pc, val);
    }
};

const Flags = packed struct(u8) {
    carry: bool = false,
    zero: bool = false,
    interrupt_disabled: bool = false,
    decimal_mode: bool = false,
    break_flag: bool = false,
    _reserved: bool = true,
    overflow: bool = false,
    negative: bool = false,
};
