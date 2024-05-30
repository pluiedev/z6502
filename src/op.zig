const std = @import("std");
const Cpu = @import("cpu.zig").Cpu;

pub const Op = struct {
    opcode: Opcode,
    addressing_mode: AddressingMode,

    pub fn init(cpu: *Cpu) ?Op {
        return opcode_table[cpu.read_and_bump_pc(u8)];
    }

    pub fn run(self: Op, cpu: *Cpu) void {
        const Input = union(enum) {
            addr: u16,
            offset: i8,
            value: u8,
        };

        const input: ?Input = switch (self.addressing_mode) {
            .zero_page => Input{ .addr = cpu.read_and_bump_pc(u8) },
            .zero_page_x => Input{ .addr = cpu.read_and_bump_pc(u8) +% cpu.x },
            .zero_page_x_ind => Input{ .addr = cpu.read(u16, cpu.read_and_bump_pc(u8) +% cpu.x) },
            .zero_page_ind_y => Input{ .addr = cpu.read(u16, cpu.read_and_bump_pc(u8)) +% cpu.y },
            .absolute => Input{ .addr = cpu.read_and_bump_pc(u16) },
            .absolute_x => Input{ .addr = cpu.read_and_bump_pc(u16) +% cpu.x },
            .absolute_y => Input{ .addr = cpu.read_and_bump_pc(u16) +% cpu.y },
            .relative => Input{ .offset = cpu.read_and_bump_pc(i8) },
            .indirect => Input{ .addr = cpu.read(u16, cpu.read_and_bump_pc(u16)) },
            .immediate => Input{ .value = cpu.read_and_bump_pc(u8) },
            .accumulator => Input{ .value = cpu.a },
            .implicit => null,
        };

        // TODO: set flag bits
        switch (self.opcode) {
            .ora => cpu.a |= switch (input.?) {
                .addr => |a| cpu.read(u8, a),
                .value => |v| v,
                else => unreachable,
            },
            .and_ => cpu.a &= cpu.read(u8, input.?.addr),
            .eor => cpu.a ^= cpu.read(u8, input.?.addr),
            .adc => {
                // It's MUCH more complicated than this.
                cpu.a += cpu.read(u8, input.?.addr);
            },
            .sta => cpu.write(u8, input.?.addr, cpu.a),
            .lda => cpu.a = cpu.read(u8, input.?.addr),
            .cmp => unreachable,
            .sbc => unreachable,
            else => unreachable,
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
    sey,
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
        .sey,
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
