const std = @import("std");
const Cpu = @import("cpu.zig").Cpu;

pub const Op = struct {
    opcode: Opcode,
    addressing_mode: AddressingMode,

    pub fn init(cpu: *Cpu) ?Op {
        return opcode_table[cpu.readAndBumpPc(u8)];
    }

    pub fn run(self: Op, cpu: *Cpu) void {
        const operand: Operand = switch (self.addressing_mode) {
            .zero_page => Operand{ .addr = cpu.readAndBumpPc(u8) },
            .zero_page_x => Operand{ .addr = cpu.readAndBumpPc(u8) +% cpu.x },
            .zero_page_x_ind => Operand{ .addr = cpu.read(u16, cpu.readAndBumpPc(u8) +% cpu.x) },
            .zero_page_ind_y => Operand{ .addr = cpu.read(u16, cpu.readAndBumpPc(u8)) +% cpu.y },
            .absolute => Operand{ .addr = cpu.readAndBumpPc(u16) },
            .absolute_x => Operand{ .addr = cpu.readAndBumpPc(u16) +% cpu.x },
            .absolute_y => Operand{ .addr = cpu.readAndBumpPc(u16) +% cpu.y },
            .relative => Operand{ .offset = cpu.readAndBumpPc(i8) },
            .indirect => Operand{ .addr = cpu.read(u16, cpu.readAndBumpPc(u16)) },
            .immediate => Operand{ .value = cpu.readAndBumpPc(u8) },
            .accumulator => Operand{ .accumulator = void{} },
            .implicit => Operand{ .none = void{} },
        };

        switch (self.opcode) {
            .ora => cpu.setA(cpu.a | operand.readFrom(u8, cpu)),
            .and_ => cpu.setA(cpu.a & operand.readFrom(u8, cpu)),
            .eor => cpu.setA(cpu.a ^ operand.readFrom(u8, cpu)),
            .adc => {
                // TODO: It's MUCH more complicated than this.
                cpu.a += operand.readFrom(u8, cpu);
                cpu.updateNZFlags(cpu.a);
            },
            .cmp => unreachable,
            .sbc => unreachable,
            .cpx => unreachable,
            .cpy => unreachable,

            .asl => {
                const new, const overflow = @shlWithOverflow(operand.readFrom(u8, cpu), 1);

                operand.writeTo(u8, cpu, new);
                cpu.updateNZFlags(new);
                cpu.flags.carry = overflow != 0;
            },
            .rol => {
                var new, const overflow = @shlWithOverflow(operand.readFrom(u8, cpu), 1);
                new |= @intFromBool(cpu.flags.carry);

                operand.writeTo(u8, cpu, new);
                cpu.updateNZFlags(new);
                cpu.flags.carry = overflow != 0;
            },
            .lsr => {
                // There's no @shrWithOverflow :(
                var new = operand.readFrom(u8, cpu);
                const overflow = new & 0b1;
                new >>= 1;

                operand.writeTo(u8, cpu, new);
                cpu.updateNZFlags(new);
                cpu.flags.carry = overflow != 0;
            },
            .ror => {
                var new = operand.readFrom(u8, cpu);
                const overflow = new & 0b1;
                new >>= 1;
                // ugh.
                const carry: u8 = @intCast(@intFromBool(cpu.flags.carry));
                new |= carry << 7;

                operand.writeTo(u8, cpu, new);
                cpu.updateNZFlags(new);
                cpu.flags.carry = overflow != 0;
            },
            .dec => {
                const new = operand.readFrom(u8, cpu) -% 1;
                operand.writeTo(u8, cpu, new);
                cpu.updateNZFlags(new);
            },
            .inc => {
                const new = operand.readFrom(u8, cpu) +% 1;
                operand.writeTo(u8, cpu, new);
                cpu.updateNZFlags(new);
            },
            .inx => cpu.setX(cpu.x +% 1),
            .iny => cpu.setY(cpu.y +% 1),
            .dex => cpu.setX(cpu.x -% 1),
            .dey => cpu.setY(cpu.y -% 1),

            .bit => {
                const input = operand.readFrom(u8, cpu);
                cpu.flags.negative = input & 0b10000000 != 0;
                cpu.flags.overflow = input & 0b01000000 != 0;
                cpu.flags.zero = cpu.a & input == 0;
            },
            .jmp => cpu.pc = operand.readFrom(u16, cpu),

            .bpl => cpu.branch(!cpu.flags.negative, operand.offset),
            .bmi => cpu.branch(cpu.flags.negative, operand.offset),
            .bvc => cpu.branch(!cpu.flags.overflow, operand.offset),
            .bvs => cpu.branch(cpu.flags.overflow, operand.offset),
            .bcc => cpu.branch(!cpu.flags.carry, operand.offset),
            .bcs => cpu.branch(cpu.flags.carry, operand.offset),
            .bne => cpu.branch(!cpu.flags.zero, operand.offset),
            .beq => cpu.branch(cpu.flags.zero, operand.offset),

            .php => cpu.saveFlags(true),
            .plp => cpu.restoreFlags(),
            .pha => cpu.stackPush(u8, cpu.a),
            .pla => cpu.setA(cpu.stackPop(u8)),

            .clc => cpu.flags.carry = false,
            .sec => cpu.flags.carry = true,
            .cli => cpu.flags.interrupt_disabled = false,
            .sei => cpu.flags.interrupt_disabled = true,
            .clv => cpu.flags.overflow = false,
            .cld => cpu.flags.decimal_mode = false,
            .sed => cpu.flags.decimal_mode = true,

            .sta => operand.writeTo(u8, cpu, cpu.a),
            .lda => cpu.setA(operand.readFrom(u8, cpu)),
            .stx => operand.writeTo(u8, cpu, cpu.x),
            .ldx => cpu.setX(operand.readFrom(u8, cpu)),
            .sty => operand.writeTo(u8, cpu, cpu.y),
            .ldy => cpu.setY(operand.readFrom(u8, cpu)),
            .txa => cpu.setA(cpu.x),
            .tax => cpu.setX(cpu.a),
            .tay => cpu.setY(cpu.a),
            .tya => cpu.setA(cpu.y),
            .txs => cpu.sp = cpu.x,
            .tsx => cpu.setX(cpu.sp),

            .brk => {
                cpu.stackPush(u16, cpu.pc + 2); // leave space for break marking
                cpu.saveFlags(true);
                // Jump to interrupt vector
                cpu.pc = cpu.read(u16, 0xfffe);
            },
            .rti => {
                cpu.restoreFlags();
                cpu.pc = cpu.stackPop(u16);
            },
            .jsr => {
                cpu.stackPush(u16, cpu.pc);
                cpu.pc = operand.readFrom(u16, cpu);
            },
            .rts => cpu.pc = cpu.stackPop(u16),

            .nop => {}, // literally nothing
        }
    }

    pub fn format(value: Op, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;

        const op = if (value.opcode == .and_) "AND" else a: {
            var buf: [3]u8 = undefined;
            break :a std.ascii.upperString(&buf, @tagName(value.opcode));
        };

        const am = switch (value.addressing_mode) {
            .zero_page => "$xx",
            .zero_page_x => if (value.opcode == .ldx or value.opcode == .stx) "$xx, Y" else "$xx, X",
            .zero_page_x_ind => "($xx, X)",
            .zero_page_ind_y => "($xx), Y",
            .absolute => "$xxxx",
            .absolute_x => "$xxxx, X",
            .absolute_y => "$xxxx, Y",
            .relative => "±rr",
            .indirect => "($xxxx)",
            .immediate => "#xx",
            .accumulator => "A",
            .implicit => "",
        };

        try writer.writeAll(op);
        try writer.writeAll(" ");
        try writer.writeAll(am);
    }
};

const Operand = union(enum) {
    addr: u16,
    value: u8,
    offset: i8,
    accumulator: void,
    none: void,

    fn readFrom(self: Operand, comptime T: type, cpu: *const Cpu) T {
        return switch (self) {
            .addr => |a| cpu.read(T, a),
            .accumulator => @intCast(cpu.a),
            .value => |v| @intCast(v),
            else => unreachable,
        };
    }
    fn writeTo(self: Operand, comptime T: type, cpu: *Cpu, value: T) void {
        return switch (self) {
            .addr => |a| cpu.write(T, a, value),
            .accumulator => cpu.a = @intCast(value),
            else => unreachable,
        };
    }
};

const Opcode = enum {
    ora,
    and_,
    eor,
    adc,
    sta,
    lda,
    cmp,
    sbc,

    asl,
    rol,
    lsr,
    ror,
    stx,
    ldx,
    dec,
    inc,

    txa,
    tax,
    dex,
    nop,

    bit,
    jmp,
    sty,
    ldy,
    cpy,
    cpx,

    bpl,
    bmi,
    bvc,
    bvs,
    bcc,
    bcs,
    bne,
    beq,

    php,
    plp,
    pha,
    pla,
    dey,
    tay,
    iny,
    inx,

    clc,
    sec,
    cli,
    sei,
    tya,
    clv,
    cld,
    sed,

    txs,
    tsx,
    brk,
    jsr,
    rti,
    rts,
};

const AddressingMode = enum {
    zero_page,
    zero_page_x,
    zero_page_x_ind,
    zero_page_ind_y,
    absolute,
    absolute_x,
    absolute_y,
    relative,
    indirect,
    immediate,
    accumulator,
    implicit,
};
const opcode_table: [256]?Op = table: {
    var table: [256]?Op = [_]?Op{null} ** 256;

    // Based on https://llx.com/Neil/a2/opcodes.html

    // "Group 1", opcode = aaabbb01 //
    for ([_]Opcode{
        .ora,
        .and_,
        .eor,
        .adc,
        .sta,
        .lda,
        .cmp,
        .sbc,
    }, 0..) |op, op_idx| {
        for ([_]AddressingMode{
            .zero_page_x_ind,
            .zero_page,
            .immediate,
            .absolute,
            .zero_page_ind_y,
            .zero_page_x,
            .absolute_y,
            .absolute_x,
        }, 0..) |am, am_idx| {
            // STA # doesn't exist. :(
            if (op == .sta and am == .immediate) continue;

            table[op_idx << 5 | am_idx << 2 | 0b01] = .{
                .opcode = op,
                .addressing_mode = am,
            };
        }
    }

    // "Group 2", opcode = aaabbb01 //
    for ([_]Opcode{
        .asl,
        .rol,
        .lsr,
        .ror,
        .stx,
        .ldx,
        .dec,
        .inc,
    }, 0..) |op, op_idx| {
        for ([_]?AddressingMode{
            .immediate,
            .zero_page,
            .accumulator,
            .absolute,
            null,
            .zero_page_x,
            null,
            .absolute_x,
        }, 0..) |am, am_idx| {
            if (am == null) continue;

            // Only LDX # takes an immediate.
            if (op != .ldx and am == .immediate) continue;
            // STX abs,Y doesn't exist :(
            if (op == .stx and am == .absolute_x) continue;

            // These do not take accumulator inputs and mean something else
            const opp: ?Opcode = if (am == .accumulator)
                switch (op) {
                    .stx => .txa,
                    .ldx => .tax,
                    .dec => .dex,
                    .inc => .nop,
                    else => null,
                }
            else
                null;

            const amm = if (opp != null)
                .implicit
            else if (op == .ldx and am == .absolute_x)
                .absolute_y
            else
                am.?;

            table[op_idx << 5 | am_idx << 2 | 0b10] = .{
                .opcode = opp orelse op,
                .addressing_mode = amm,
            };
        }
    }

    // "Group 3", opcode = aaabbb00 //
    for ([_]Opcode{
        .sty,
        .ldy,
        .cpy,
        .cpx,
    }, 0b100..) |op, op_idx| {
        for ([_]?AddressingMode{
            .immediate,
            .zero_page,
            null,
            .absolute,
            null,
            .zero_page_x,
            null,
            .absolute_x,
        }, 0..) |am, am_idx| {
            if (am == null) continue;

            // Writing to an immediate value is nonsensical,
            // but STY abs,X also doesn't exist.. why?
            if (op == .sty and (am == .immediate or am == .absolute_x)) continue;

            // Huh??
            if ((op == .cpy or op == .cpx) and (am == .zero_page_x or am == .absolute_x)) continue;

            table[op_idx << 5 | am_idx << 2 | 0b00] = .{
                .opcode = op,
                .addressing_mode = am.?,
            };
        }
    }

    // Branches
    for ([_]Opcode{
        .bpl,
        .bmi,
        .bvc,
        .bvs,
        .bcc,
        .bcs,
        .bne,
        .beq,
    }, 0..) |op, op_idx| {
        table[0x10 + op_idx * 0x20] = .{
            .opcode = op,
            .addressing_mode = .relative,
        };
    }

    // Miscellanea
    for ([_]Opcode{
        .php,
        .plp,
        .pha,
        .pla,
        .dey,
        .tay,
        .iny,
        .inx,
    }, 0..) |op, op_idx| {
        table[0x08 + op_idx * 0x20] = .{
            .opcode = op,
            .addressing_mode = .implicit,
        };
    }

    for ([_]Opcode{
        .clc,
        .sec,
        .cli,
        .sei,
        .tya,
        .clv,
        .cld,
        .sed,
    }, 0..) |op, op_idx| {
        table[0x18 + op_idx * 0x20] = .{
            .opcode = op,
            .addressing_mode = .implicit,
        };
    }

    table[0x9a] = .{
        .opcode = .txs,
        .addressing_mode = .implicit,
    };
    table[0xba] = .{
        .opcode = .tsx,
        .addressing_mode = .implicit,
    };

    table[0x24] = .{
        .opcode = .bit,
        .addressing_mode = .zero_page,
    };
    table[0x2c] = .{
        .opcode = .bit,
        .addressing_mode = .absolute,
    };
    table[0x4c] = .{
        .opcode = .jmp,
        .addressing_mode = .absolute,
    };
    table[0x6c] = .{
        .opcode = .jmp,
        .addressing_mode = .indirect,
    };

    table[0x00] = .{
        .opcode = .brk,
        .addressing_mode = .implicit,
    };
    table[0x20] = .{
        .opcode = .jsr,
        .addressing_mode = .absolute,
    };
    table[0x40] = .{
        .opcode = .rti,
        .addressing_mode = .implicit,
    };
    table[0x60] = .{
        .opcode = .rts,
        .addressing_mode = .implicit,
    };

    break :table table;
};

test "opcode table" {
    var buf = [_]u8{0} ** 12;

    std.debug.print("|----", .{});
    for (0..16) |_| {
        std.debug.print("|" ++ "-" ** 14, .{});
    }
    std.debug.print("|\n", .{});

    std.debug.print("|    ", .{});
    for (0..16) |lower_nibble| {
        const s = try std.fmt.bufPrint(&buf, "x{X}", .{lower_nibble});
        std.debug.print("| {s:^12} ", .{s});
    }
    std.debug.print("|\n", .{});

    std.debug.print("|----", .{});
    for (0..16) |_| {
        std.debug.print("|" ++ "-" ** 14, .{});
    }
    std.debug.print("|\n", .{});

    for (0..16) |upper_nibble| {
        std.debug.print("| {X}x ", .{upper_nibble});
        for (0..16) |lower_nibble| {
            const opcode = opcode_table[upper_nibble << 4 | lower_nibble];

            const s = if (opcode) |op|
                try std.fmt.bufPrint(&buf, "{any}", .{op})
            else
                "";

            std.debug.print("| {s:<12} ", .{s});
        }
        std.debug.print("|\n", .{});
    }

    std.debug.print("|----", .{});
    for (0..16) |_| {
        std.debug.print("|" ++ "-" ** 14, .{});
    }
    std.debug.print("|\n", .{});
}
