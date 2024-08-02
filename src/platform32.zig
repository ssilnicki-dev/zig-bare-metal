pub const BusType: type = u32;
pub const FieldShiftType = u5;
pub const FieldWidthType = FieldShiftType;

pub fn SwitchInterface(comptime T: type, port: BusType) type {
    return struct {
        pub fn enable(self: *const T, value: anytype) void {
            _ = self; // autofix
            value.enable(port);
        }
        pub fn disable(self: *const T, value: anytype) void {
            _ = self; // autofix
            value.disable(port);
        }
    };
}

pub fn PeripheryInterface(comptime T: type) type {
    return struct {
        pub fn set(self: *const T, value: anytype) void {
            comptime if (!@hasField(@TypeOf(self.*), "port"))
                @compileError("interface user has to provide field port (u32 or Bustype)");
            comptime {
                switch (@typeInfo(@TypeOf(value))) {
                    .Enum => {},
                    .Struct => {
                        if (!@hasField(@TypeOf(value), "value") or @typeInfo(@TypeOf(@field(value, "value"))) != .Int)
                            @compileError("unsupported type");
                    },
                    else => @compileError("unsupported type"),
                }
            }
            value.writeTo(self.port);
        }

        pub fn get(self: *const T, output_type: type) output_type {
            const ret: output_type = undefined;
            comptime if (!@hasField(@TypeOf(self.*), "port"))
                @compileError("interface user has to provide field port (u32 or Bustype)");
            comptime switch (@typeInfo(output_type)) {
                .Enum => {
                    if (@typeInfo(output_type).Enum.fields.len < (1 << @bitSizeOf(@TypeOf(@intFromEnum(ret))))) {
                        @compileLog(output_type, "Enum must describe all possible values");
                    }
                },
                .Struct => {
                    if (!@hasField(output_type, "value") or @typeInfo(@TypeOf(@field(output_type{}, "value"))) != .Int)
                        @compileLog(output_type, "unsupported type");
                },
                else => @compileLog(output_type, "unsupported type"),
            };
            return ret.readFrom(self.port);
        }
    };
}

pub fn PlatformPeripheryWriteInterface(comptime T: type) type {
    return struct {
        pub fn writeReg(self: *const T, reg: comptime_int, index: comptime_int, buf: []const u8) !void {
            const reg_ptr = @as(*volatile BusType, @ptrFromInt(self.port + reg + index));
            switch (buf.len) {
                1 => reg_ptr.* = buf[0],
                2 => reg_ptr.* = @as(*const u16, @alignCast(@ptrCast(buf.ptr))).*,
                4 => reg_ptr.* = @as(*const BusType, @alignCast(@ptrCast(buf.ptr))).*,
                else => unreachable,
            }
        }
    };
}

pub fn PlatformPeripheryReadInterface(comptime T: type) type {
    return struct {
        pub fn readReg(self: *const T, reg: comptime_int, index: comptime_int, buf: []u8) !void {
            const reg_ptr = @as(*volatile BusType, @ptrFromInt(self.port + reg + index));
            switch (buf.len) {
                1 => @as(*volatile u8, &buf[0]).* = @truncate(reg_ptr.*),
                2 => @as(*volatile u16, @alignCast(@ptrCast(buf.ptr))).* = @truncate(reg_ptr.*),
                4 => @as(*volatile BusType, @alignCast(@ptrCast(buf.ptr))).* = reg_ptr.*,
                else => unreachable,
            }
        }
    };
}

pub fn PlatformPeripheryInterface(comptime T: type) type {
    return struct {
        pub usingnamespace PlatformPeripheryReadInterface(T);
        pub usingnamespace PlatformPeripheryWriteInterface(T);
    };
}

pub const Field = struct {
    pub const RwType = enum {
        ReadOnly,
        WriteOnly,
        ReadWrite,
    };
    reg: BusType,
    width: FieldWidthType,
    rw: RwType = .ReadWrite,
    shift: FieldShiftType,

    pub fn set(self: Field, value: BusType) void {
        const addr: *volatile BusType = @ptrFromInt(self.reg);
        if (self.rw == .ReadOnly)
            return;

        if (self.rw == .WriteOnly) {
            addr.* = value << self.shift;
            return;
        }
        addr.* = addr.* & self.getResetMask() | ((value << self.shift) & self.getMask());
    }
    pub fn get(self: Field) BusType {
        const addr: *volatile BusType = @ptrFromInt(self.reg);
        return ((addr.* & self.getMask()) >> self.shift);
    }

    pub fn isAsserted(self: Field) bool {
        return self.get() != 0;
    }
    pub fn isCleared(self: Field) bool {
        return self.get() == 0;
    }

    fn getMask(self: Field) BusType {
        return ((@as(BusType, 1) << self.width) - 1) << self.shift;
    }

    fn getResetMask(self: Field) BusType {
        return ~self.getMask();
    }
    pub fn Interface(comptime T: type, reg: anytype, shift: FieldShiftType) type {
        return struct {
            pub inline fn getValueBitSize(self: T) BusType {
                return switch (@typeInfo(T)) {
                    .Enum => @bitSizeOf(@TypeOf(@intFromEnum(self))),
                    else => @bitSizeOf(@TypeOf(self.value)),
                };
            }

            pub fn enable(self: T, port: BusType) void {
                self.getField(port, 1).set(1);
            }

            pub fn disable(self: T, port: BusType) void {
                self.getField(port, 1).set(0);
            }

            pub inline fn getField(self: T, port: BusType, width: ?comptime_int) Field {
                const type_info = @typeInfo(T);
                return Field{
                    .reg = port + switch (@typeInfo(@TypeOf(reg))) {
                        .Enum => @intFromEnum(reg),
                        else => reg,
                    },
                    .shift = shift,
                    .rw = switch (@hasField(T, "rw")) {
                        true => T.rw,
                        false => Field.RwType.ReadWrite,
                    },
                    .width = width orelse switch (type_info) {
                        .Enum => @bitSizeOf(@TypeOf(@intFromEnum(self))),
                        else => @bitSizeOf(@TypeOf(self.value)),
                    },
                };
            }
            pub fn writeTo(self: T, port: BusType) void {
                const port_bit_size = @bitSizeOf(@TypeOf(port));
                const register: @TypeOf(port) = switch (@typeInfo(@TypeOf(reg))) {
                    .Enum => @intFromEnum(reg),
                    else => reg,
                };
                switch (self.getValueBitSize()) {
                    0...(port_bit_size - 1) => {
                        const field = self.getField(port, null);
                        switch (@typeInfo(T)) {
                            .Enum => field.set(@intFromEnum(self)),
                            else => field.set(self.value),
                        }
                    },
                    port_bit_size => {
                        @as(*volatile BusType, @ptrFromInt(port + register)).* = switch (@typeInfo(T)) {
                            .Enum => @intFromEnum(self),
                            else => self.value,
                        };
                    },
                    else => @compileError("target port bit size overrun"),
                }
            }

            pub fn readFrom(self: T, port: BusType) T {
                const port_bit_size = @bitSizeOf(@TypeOf(port));
                const value_bit_size = self.getValueBitSize();
                comptime if ((shift + value_bit_size) > port_bit_size)
                    @compileLog(T, shift, value_bit_size, port_bit_size, "out of bounds");
                const register: @TypeOf(port) = switch (@typeInfo(@TypeOf(reg))) {
                    .Enum => @intFromEnum(reg),
                    else => reg,
                };

                switch (value_bit_size) {
                    0...(port_bit_size - 1) => {
                        const value = self.getField(port, null).get();
                        return switch (@typeInfo(T)) {
                            .Enum => @enumFromInt(value),
                            else => T{ .value = @truncate(value) },
                        };
                    },
                    port_bit_size => {
                        const value = @as(*volatile BusType, @ptrFromInt(port + register)).*;
                        return switch (@typeInfo(T)) {
                            .Enum => @intFromEnum(value),
                            else => T{ .value = value },
                        };
                    },
                    else => @compileError("target port bit size overrun"),
                }
            }
        };
    }
};
