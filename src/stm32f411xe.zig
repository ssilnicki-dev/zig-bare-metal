const BusType: type = u32;
const FieldShiftType = u5;
const FieldWidthType = FieldShiftType;

const hsi_fq_hz: u32 = 16000000;
var hse_fq_hz: u32 = undefined;

const bus: struct {
    ahb1: struct {
        const base: BusType = 0x40020000;
        rcc: RCC = .{ .port = 0x3800 + base },
        gpioa: GPIO = .{ .port = 0x0000 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 0 } },
        gpiob: GPIO = .{ .port = 0x0400 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 1 } },
        gpioc: GPIO = .{ .port = 0x0800 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 2 } },
        gpiod: GPIO = .{ .port = 0x0C00 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 3 } },
        gpioe: GPIO = .{ .port = 0x1000 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 4 } },
        gpioh: GPIO = .{ .port = 0x1C00 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 7 } },
    } = .{},
} = .{};

pub const gpioa = bus.ahb1.gpioa;
pub const gpiob = bus.ahb1.gpiob;
pub const gpioc = bus.ahb1.gpioc;
pub const gpiod = bus.ahb1.gpiod;
pub const gpioe = bus.ahb1.gpioe;
pub const gpioh = bus.ahb1.gpioh;
pub const rcc = bus.ahb1.rcc;

const Field = struct {
    pub const RwType = enum {
        ReadOnly,
        WriteOnly,
        ReadWrite,
    };
    reg: BusType,
    width: FieldWidthType,
    rw: RwType = .ReadWrite,
    shift: FieldShiftType,

    fn set(self: Field, value: BusType) void {
        const addr: *volatile BusType = @ptrFromInt(self.reg);
        if (self.rw == .ReadOnly)
            return;

        if (self.rw == .WriteOnly) {
            addr.* = value << self.shift;
            return;
        }
        addr.* = addr.* & self.getResetMask() | ((value << self.shift) & self.getMask());
    }
    fn get(self: Field) BusType {
        const addr: *volatile BusType = @ptrFromInt(self.reg);
        return ((addr.* & self.getMask()) >> self.shift);
    }

    fn isAsserted(self: Field) bool {
        return self.get() != 0;
    }
    fn isCleared(self: Field) bool {
        return self.get() == 0;
    }

    fn getMask(self: Field) BusType {
        return ((@as(BusType, 1) << self.width) - 1) << self.shift;
    }

    fn getResetMask(self: Field) BusType {
        return ~self.getMask();
    }
};

const Register = struct {
    addr: BusType,
    inline fn reset(self: *const Register) void {
        self.set(0x0);
    }
    fn set(self: *const Register, value: BusType) void {
        @as(*volatile BusType, @ptrFromInt(self.addr)).* = value;
    }
    fn get(self: *const Register) BusType {
        return @as(*volatile BusType, @ptrFromInt(self.addr)).*;
    }

    fn monitorBits(self: *const Register, comptime bits: []const FieldShiftType) u5 {
        comptime if (bits.len == 0)
            @compileError("Have to specify at least one bit for monitoring");

        var mask: BusType = 0;
        for (bits) |bit| mask |= @as(@TypeOf(mask), 1) << bit;

        while (true) {
            const bit = @bitSizeOf(@TypeOf(mask)) - @clz(self.get() & mask);
            if (bit != 0)
                return @truncate(bit - 1);
        }
    }
};

const RCC = struct {
    port: BusType,
    fn getReg(self: *const RCC, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const ClockSource = enum { HSI, HSE };
    const ClockMuxer = struct {
        src: FieldDesc,
        rdy: ?FieldDesc = null,
        const FieldDesc = struct {
            reg: Reg,
            shift: FieldShiftType,
            width: FieldWidthType = 1,
        };
    };
    const PeripherySwitch = struct {
        en_reg: Reg,
        shift: FieldShiftType,
    };

    const HSEMode = enum { Crystal };

    pub fn enableHSE(self: *const RCC, mode: HSEMode, fq: u32) void {
        _ = self; // autofix
        _ = mode; // autofix
        hse_fq_hz = fq;
    }

    fn enablePeriphery(self: *const RCC, periph_switch: PeripherySwitch) void {
        (Field{ .reg = self.getReg(periph_switch.en_reg), .rw = .ReadWrite, .shift = periph_switch.shift, .width = 1 }).set(1);
    }

    fn disablePeriphery(self: *const RCC, periph_switch: PeripherySwitch) void {
        (Field{ .reg = self.getReg(periph_switch.en_reg), .rw = .ReadWrite, .shift = periph_switch.shift, .width = 1 }).reset();
    }

    const Reg = enum(BusType) {
        CR = 0x0,
        PLLCFGR = 0x04,
        CFGR = 0x08,
        CIR = 0x0C,
        AHB1RSTR = 0x10,
        AHB2RSTR = 0x14,
        APB1RSTR = 0x20,
        APB2RSTR = 0x24,
        AHB1ENR = 0x30,
        AHB2ENR = 0x34,
        APB1ENR = 0x40,
        APB2ENR = 0x44,
        AHB1LPENR = 0x50,
        AHB2LPENR = 0x54,
        APB1LPENR = 0x60,
        APB2LPENR = 0x64,
        BDCR = 0x70,
        CSR = 0x74,
        SSCGR = 0x80,
        PLLI2SCFGR = 0x84,
        DCKCFGR = 0x8C,
    };
};

const GPIO = struct {
    port: BusType,
    rcc_switch: RCC.PeripherySwitch,
    fn getReg(self: *const GPIO, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const Reg = enum(BusType) {
        MODER = 0x0,
        OTYPER = 0x4,
        OSPEEDR = 0x8,
        PUPDR = 0xC,
        IDR = 0x10,
        BRSR = 0x18,
        AFR = 0x20,
    };
    const MODE = enum(u2) { Input = 0, Output = 1, AltFunc = 2, Analog = 3 };
    const OTYPE = enum(u1) { PushPull = 0, OpenDrain = 1 };
    const OSPEED = enum(u2) { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
    const PUPD = enum(u2) { Disabled = 0, PullUp = 1, PullDown = 2, Reserved = 3 };

    pub fn pin(self: *const GPIO, nr: u4) Pin {
        return .{ .gpio = self, .pin = nr };
    }

    const Pin = struct {
        gpio: *const GPIO,
        pin: u4,

        pub fn configure(self: *const Pin, mode: MODE, otype: OTYPE, ospeed: OSPEED, pupd: PUPD, af: u4) void {
            if (mode == .AltFunc)
                rcc.enablePeriphery(self.gpio.rcc_switch);
            self.getMODER().set(@intFromEnum(mode));
            self.getOTYPER().set(@intFromEnum(otype));
            self.getOSPEEDR().set(@intFromEnum(ospeed));
            self.getPUPDR().set(@intFromEnum(pupd));
            if (mode == .AltFunc)
                self.getAFR().set(af);
        }
        pub fn isAsserted(self: Pin) bool {
            return !(self.getIDR().get() == 0);
        }
        pub fn assert(self: Pin) void {
            self.getBSR().set(1);
        }
        pub fn reset(self: Pin) void {
            self.getBRR().set(1);
        }
        fn getMODER(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.MODER), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin) * 2, .width = 2 };
        }
        fn getOTYPER(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.OTYPER), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin), .width = 1 };
        }
        fn getOSPEEDR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.OSPEEDR), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin) * 2, .width = 2 };
        }
        fn getPUPDR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.PUPDR), .rw = .ReadWrite, .shift = @as(FieldShiftType, self.pin) * 2, .width = 2 };
        }
        fn getAFR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.AFR) + (self.pin / 8) * 4, .rw = .ReadWrite, .shift = (@as(FieldShiftType, self.pin) % 8) * 4, .width = 4 };
        }
        fn getBSR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.BRSR), .rw = .WriteOnly, .shift = @as(FieldShiftType, self.pin), .width = 1 };
        }
        fn getBRR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.BRSR), .rw = .WriteOnly, .shift = 0x10 + @as(FieldShiftType, self.pin), .width = 1 };
        }
        fn getIDR(self: Pin) Field {
            return .{ .reg = self.gpio.getReg(.IDR), .rw = .ReadOnly, .shift = @as(FieldShiftType, self.pin), .width = 1 };
        }
    };
};
