const platform = @import("platform32.zig");
const BusType = platform.BusType;
const setting = @import("setting.zig");

pub const SCS = struct {
    port: BusType,

    pub usingnamespace setting.ControllerInterface(@This());
    pub usingnamespace platform.PlatformPeripheryReadInterface(@This());
    pub fn writeReg(self: *const SCS, reg: comptime_int, index: comptime_int, buf: []const u8) !void {
        var value = switch (reg) {
            else => @as(*const BusType, @alignCast(@ptrCast(buf.ptr))).*,
            @intFromEnum(Reg.AIRCR) => (@as(*const BusType, @alignCast(@ptrCast(buf.ptr))).* & 0xFFFF) | (0x5FA << 16), // VECTKEY
        };
        _ = &value;
        @as(*volatile BusType, @ptrFromInt(self.port + reg + index)).* = value;
    }

    pub const UseFpu = CPACR.FPU;
    pub const VtorPlacement = VTOR.Placement;
    pub const Trace = DEMCR.Trace;
    pub const InterruptGroupingPriority = AIRCR.PRIGROUP;

    const Reg = enum(BusType) {
        ACTLR = 0x08, // Auxiliary control register
        CPUID = 0xD00, // CPUID base register
        ICSR = 0xD04, // Interrupt control and state register
        VTOR = 0xD08, // Vector table offset register
        AIRCR = 0xD0C, // Application interrupt and reset control register
        SCR = 0xD10, // System control register
        CCR = 0xD14, // Configuration and control register
        SHPR1 = 0xD18, // System handler priority register
        SHPR2 = 0xD1C, // System handler priority register
        SHPR3 = 0xD20, // System handler priority register
        SHCSR = 0xD24, // System handler control and state register
        CFSR_MMSR_BFSR_UFSR = 0xD28, // Configurable fault status register
        HFSR = 0xD2C, // Hard fault status register
        MMAR = 0xD34, // Memory management fault address register
        BFAR = 0xD38, // Bus fault address register
        AFSR = 0xD3C, // Auxiliary fault status register
        CPACR = 0xD88, // Coprocessor Access Control Register
        DHCSR = 0xDF0, //  Debug Halting Control and Status Register */
        DCRSR = 0xDF4, //  Debug Core Register Selector Register */
        DCRDR = 0xDF8, //  Debug Core Register Data Register */
        DEMCR = 0xDFC, //  Debug Exception and Monitor Control Register */
    };
};

const AIRCR = struct {
    const reg = @intFromEnum(SCS.Reg.AIRCR);
    const PRIGROUP = struct {
        value: u3 = undefined,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, u32, 0, 8);
    };
};

const CPACR = struct {
    const reg = @intFromEnum(SCS.Reg.CPACR);
    const FPU = struct {
        value: u4 = undefined,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, u32, 0, 20);
    };
};

const VTOR = struct {
    const reg = @intFromEnum(SCS.Reg.VTOR);
    const Placement = enum(u1) {
        Sysram = 1,
        Flash = 0,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, u32, 0, 29);
    };
};

const DEMCR = struct {
    const reg = @intFromEnum(SCS.Reg.DEMCR);
    pub const Trace = enum(u1) {
        Enabled = 1,
        Disabled = 0,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, u32, 0, 24);
    };
};
