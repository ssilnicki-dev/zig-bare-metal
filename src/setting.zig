fn GetValueType(comptime T: type) type {
    return comptime switch (@typeInfo(T)) {
        .Enum => T,
        else => (T{}).getRetType(),
    };
}

fn PrivateInterface(comptime T: type, reg_type: type) type {
    return struct {
        inline fn getMask(self: *const T) reg_type {
            const value_type = GetValueType(@TypeOf(self.*));
            return switch (@typeInfo(value_type)) {
                .Enum => (1 << @bitSizeOf(@TypeOf(@intFromEnum(((struct { foo: value_type = undefined }){}).foo)))) - 1,
                else => (1 << @bitSizeOf(value_type)) - 1,
            };
        }
        fn getRetType(self: T) type {
            return comptime switch (@typeInfo(@TypeOf(self))) {
                .Struct => blk: {
                    if (@hasField(T, "value"))
                        break :blk @TypeOf((T{}).value);
                    break :blk reg_type;
                },
                else => @compileLog("unsupported type", T),
            };
        }
    };
}

pub fn PackedStructReadInterface(comptime T: type, reg: comptime_int, index: comptime_int) type {
    const ti = @typeInfo(T);
    comptime if (ti != .Struct) @compileLog("only struct type supported", T);
    comptime if (ti.Struct.layout != .@"packed") @compileLog("only packed struct supported", ti.Struct.layout);
    comptime if (@bitSizeOf(ti.Struct.backing_integer.?) % 8 != 0) @compileLog("struct must at leas byte allign", ti.Struct.backing_integer.?);

    return struct {
        pub fn read(self: *T, periph: anytype) !void {
            const bytes = @bitSizeOf(@typeInfo(T).Struct.backing_integer.?) / 8;
            try periph.readReg(reg, index, @as(*[bytes]u8, @ptrCast(self))[0..bytes]);
        }
    };
}
pub fn PackedStructWriteInterface(comptime T: type, reg: comptime_int, index: comptime_int) type {
    const ti = @typeInfo(T);
    comptime if (ti != .Struct) @compileLog("only struct type supported", T);
    comptime if (ti.Struct.layout != .@"packed") @compileLog("only packed struct supported", ti.Struct.layout);
    comptime if (@bitSizeOf(ti.Struct.backing_integer.?) % 8 != 0) @compileLog("struct must at leas byte allign", ti.Struct.backing_integer.?);

    return struct {
        pub fn write(self: *const T, periph: anytype) !void {
            const bytes = @bitSizeOf(@typeInfo(T).Struct.backing_integer.?) / 8;
            try periph.writeReg(reg, index, @as(*const [bytes]u8, @ptrCast(self))[0..bytes]);
        }
    };
}
pub fn PackedStructInterface(comptime T: type, reg: comptime_int, index: comptime_int) type {
    return struct {
        pub usingnamespace PackedStructReadInterface(T, reg, index);
        pub usingnamespace PackedStructWriteInterface(T, reg, index);
    };
}

pub fn ReadInterface(comptime T: type, reg: comptime_int, reg_type: type, index: comptime_int, offset: comptime_int) type {
    return struct {
        usingnamespace PrivateInterface(T, reg_type);
        pub fn read(self: *const T, periph: anytype) !GetValueType(T) {
            const mask = self.getMask();
            var reg_value: reg_type = undefined;
            const bytes = @bitSizeOf(reg_type) / 8;
            try periph.readReg(reg, index, @as(*[bytes]u8, @ptrCast(&reg_value))[0..bytes]);

            return switch (@typeInfo(T)) {
                .Enum => @enumFromInt((reg_value >> offset) & mask),
                else => @truncate((reg_value >> offset) & mask),
            };
        }
    };
}

pub fn WriteInterface(comptime T: type, reg: comptime_int, reg_type: type, index: comptime_int, offset: comptime_int) type {
    return struct {
        usingnamespace PrivateInterface(T, reg_type);
        pub fn write(self: *const T, periph: anytype) !void {
            var reg_value: reg_type = undefined;
            const bytes = @bitSizeOf(reg_type) / 8;
            const buf = @as(*[bytes]u8, @ptrCast(&reg_value))[0..bytes];
            try periph.readReg(reg, index, buf);
            reg_value &= ~(self.getMask() << offset);

            reg_value |= switch (@typeInfo(T)) {
                .Enum => @as(reg_type, @intFromEnum(self.*)) << offset,
                else => @as(reg_type, self.value) << offset,
            };
            try periph.writeReg(reg, index, buf);
        }
    };
}
pub fn ReadWriteInterface(comptime T: type, reg: comptime_int, reg_type: type, index: comptime_int, offset: comptime_int) type {
    return struct {
        pub usingnamespace ReadInterface(T, reg, reg_type, index, offset);
        pub usingnamespace WriteInterface(T, reg, reg_type, index, offset);
    };
}

pub fn BitMapInterface(comptime T: type, reg: comptime_int, index: comptime_int) type {
    return struct {
        pub usingnamespace BitFieldInterface(T, reg, index, .ZeroToClear, .PreserveRegValue);
    };
}

pub fn BitFieldInterface(comptime T: type, reg: comptime_int, index: comptime_int, comptime clear_strategy: enum { ZeroToClear, AssertToClear }, comptime preserve_reg_value: enum { PreserveRegValue, IgnoreRegValue }) type {
    comptime if (@typeInfo(T) != .Enum) {
        @compileLog("only .Enum type supported for BitFieldInterface", T);
    };
    comptime if (@alignOf(@typeInfo(T).Enum.tag_type) * 8 != @bitSizeOf(@typeInfo(T).Enum.tag_type)) {
        @compileLog("only alignable types (u8, u16, u32, etc.) supported for .Enum tag type here", T, @typeInfo(T).Enum.tag_type);
    };
    comptime {
        const reg_type_bits = @bitSizeOf(@typeInfo(T).Enum.tag_type);
        for (@typeInfo(T).Enum.fields) |v| {
            if (v.value >= reg_type_bits)
                @compileLog("unsupported field value: ", v);
        }
    }

    return struct {
        const reg_type = @typeInfo(T).Enum.tag_type;
        const reg_bytes = @bitSizeOf(reg_type) / 8;

        pub fn getRegValue(periph: anytype) !reg_type {
            var reg_value: reg_type = 0;
            const buf = @as(*[reg_bytes]u8, @ptrCast(&reg_value))[0..reg_bytes];
            try periph.readReg(reg, index, buf);
            return reg_value;
        }

        pub fn clearAll(periph: anytype) !void {
            var reg_value: reg_type = 0;
            const buf = @as(*[reg_bytes]u8, @ptrCast(&reg_value))[0..reg_bytes];
            if (clear_strategy == .AssertToClear)
                reg_value = ~reg_value;
            try periph.writeReg(reg, index, buf);
        }
        pub fn assertAll(periph: anytype) !void {
            if (clear_strategy == .AssertToClear)
                @compileLog("ambiguous logic request", T);
            var reg_value: reg_type = 0;
            reg_value = ~reg_value;
            const buf = @as(*[reg_bytes]u8, @ptrCast(&reg_value))[0..reg_bytes];
            try periph.writeReg(reg, index, buf);
        }

        pub fn assertBit(self: *const T, periph: anytype) !void {
            if (clear_strategy == .AssertToClear)
                @compileLog("ambiguous logic request", T);
            var reg_value: reg_type = 0;
            const buf = @as(*[reg_bytes]u8, @ptrCast(&reg_value))[0..reg_bytes];
            if (preserve_reg_value == .PreserveRegValue) {
                try periph.readReg(reg, index, buf);
            }
            reg_value |= @as(reg_type, 1) << @truncate(@intFromEnum(self.*));
            try periph.writeReg(reg, index, buf);
        }

        pub fn clearBit(self: *const T, periph: anytype) !void {
            var reg_value: reg_type = 0;
            const bit: reg_type = @as(reg_type, 1) << @truncate(@intFromEnum(self.*));
            const buf = @as(*[reg_bytes]u8, @ptrCast(&reg_value))[0..reg_bytes];
            switch (clear_strategy) {
                .ZeroToClear => {
                    try periph.readReg(reg, index, buf);
                    reg_value &= ~bit;
                },
                .AssertToClear => reg_value = bit,
            }
            try periph.writeReg(reg, index, buf);
        }
        pub fn bitAsserted(self: *const T, periph: anytype) !bool {
            var reg_value: reg_type = undefined;
            const bit: reg_type = @as(reg_type, 1) << @truncate(@intFromEnum(self.*));
            const buf = @as(*[reg_bytes]u8, @ptrCast(&reg_value))[0..reg_bytes];
            try periph.readReg(reg, index, buf);
            return (reg_value & bit) == bit;
        }

        pub fn bitAssertedAt(self: *const T, reg_value: reg_type) bool {
            const bit: reg_type = @as(reg_type, 1) << @truncate(@intFromEnum(self.*));
            return (reg_value & bit) == bit;
        }
    };
}

pub fn ControllerInterface(comptime T: type) type {
    return struct {
        pub fn read(self: *const T, f: anytype) !GetValueType(f) {
            const value: f = undefined;
            return value.read(self);
        }
        pub fn write(self: *const T, value: anytype) !void {
            try value.write(self);
        }
        pub fn clearAll(self: *const T, value: anytype) !void {
            try value.clearAll(self);
        }
        pub fn assertAll(self: *const T, value: anytype) !void {
            try value.assertAll(self);
        }
        pub fn clear(self: *const T, value: anytype) !void {
            try value.clearBit(self);
        }
        pub fn assert(self: *const T, value: anytype) !void {
            try value.assertBit(self);
        }
        pub fn isAsserted(self: *const T, value: anytype) !bool {
            return value.bitAsserted(self);
        }
        pub fn isCleared(self: *const T, value: anytype) !bool {
            return !try value.bitAsserted(self);
        }
    };
}
