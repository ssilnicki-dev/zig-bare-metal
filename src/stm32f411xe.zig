const BusType: type = u32;
const FieldShiftType = u5;
const FieldWidthType = FieldShiftType;

const hsi_fq_hz: u32 = 16000000;
var hse_fq_hz: u32 = undefined;

const bus: struct {
    core: struct {
        dwt: DWT = .{ .port = 0xE0001000 },
        scb: SCB = .{ .port = 0xE000E000 },
    } = .{},
    ahb1: struct {
        const base: BusType = 0x40020000;
        rcc: RCC = .{ .port = 0x3800 + base },
        flash: FLASH = .{ .port = 0x3C00 + base },
        gpioa: GPIO = .{ .port = 0x0000 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 0 } },
        gpiob: GPIO = .{ .port = 0x0400 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 1 } },
        gpioc: GPIO = .{ .port = 0x0800 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 2 } },
        gpiod: GPIO = .{ .port = 0x0C00 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 3 } },
        gpioe: GPIO = .{ .port = 0x1000 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 4 } },
        gpioh: GPIO = .{ .port = 0x1C00 + base, .rcc_switch = .{ .en_reg = .AHB1ENR, .shift = 7 } },
    } = .{},
    mux: struct {
        main_pll: MUXER = .{ .src = .{ .reg = .PLLCFGR, .shift = 22 }, .values = enum(u1) { HSI = 0, HSE = 1 } },
        sys_clock: MUXER = .{ .src = .{ .reg = .CFGR, .shift = 0, .width = 2 }, .values = enum(u2) { HSI = 0, HSE = 1, PLL = 2 } },
    } = .{},
    pll: struct {
        main: PLL = .{ .cfg = .PLLCFGR, .en_bit = 24, .rdy_bit = 25 },
    } = .{},
    presc: struct {
        ahb: PRESCALER = .{ .cfg = .CFGR, .value_type = .MostBit, .width = 4, .shift = 4 },
        apb1: PRESCALER = .{ .cfg = .CFGR, .value_type = .MostBit, .width = 3, .shift = 10 },
        apb2: PRESCALER = .{ .cfg = .CFGR, .value_type = .MostBit, .width = 3, .shift = 13 },
    } = .{},
} = .{};

const PRESCALER = struct {
    cfg: RCC.Reg,
    value_type: enum { Direct, MostBit },
    width: FieldWidthType,
    shift: FieldShiftType,

    fn getMaxValue(self: *const PRESCALER) BusType {
        return comptime switch (self.value_type) {
            .Direct => (@as(BusType, 1) << self.width) / 2 + 1,
            .MostBit => @as(BusType, 1) << ((@as(BusType, 1) << self.width) / 2 + 1),
        };
    }
    fn validate(self: *const PRESCALER, comptime value: BusType) void {
        if (value > self.getMaxValue())
            @compileLog("Error: PRESCALER value overflow.", value, " > ", self.getMaxValue());
        if (value == 0)
            @compileLog("Error: PRESCALER value must be > 0.", value);
        if (self.value_type == .MostBit and @bitSizeOf(@TypeOf(value)) - @clz(value) - 1 != @ctz(value))
            @compileLog("Error: PRESCALER value is unalligned", value);
        if (value == 32)
            @compileLog("Error: PRESCALER value 32 is not supported for some reson...", value);
    }
    pub fn set(comptime self: *const PRESCALER, comptime value: BusType) void {
        comptime self.validate(value);
        const field: Field = Field{ .reg = rcc.getReg(self.cfg), .width = self.width, .shift = self.shift };
        switch (self.value_type) {
            .Direct => field.set(2 + value),
            .MostBit => field.set(@ctz(value) + 7 - @as(BusType, 1) * @min(value / 64, 1)),
        }
    }
};

pub const gpioa = bus.ahb1.gpioa;
pub const gpiob = bus.ahb1.gpiob;
pub const gpioc = bus.ahb1.gpioc;
pub const gpiod = bus.ahb1.gpiod;
pub const gpioe = bus.ahb1.gpioe;
pub const gpioh = bus.ahb1.gpioh;
pub const rcc = bus.ahb1.rcc;
pub const scb = bus.core.scb;
pub const flash = bus.ahb1.flash;
pub const mux = bus.mux;
pub const pll = bus.pll;
pub const presc = bus.presc;

pub fn udelay(us: BusType, sys_clock_hz: BusType) void {
    const cyccnt: *volatile BusType = @ptrFromInt(0xE0001004); // see DWT
    cyccnt.* = 0;

    const wasted_cycles = sys_clock_hz / 1_000_000 * us;
    while (wasted_cycles > cyccnt.*) {}
}
pub fn enableCycleCounter() void {
    bus.core.dwt.enableCycleCounter();
}

pub fn getSysClockHz() BusType {
    const sysclock_src = bus.mux.sys_clock.values;
    const main_pll_src = bus.mux.main_pll.values;

    switch (bus.mux.sys_clock.get()) {
        sysclock_src.HSE => {
            return hse_fq_hz;
        },
        sysclock_src.HSI => {
            return hsi_fq_hz;
        },
        sysclock_src.PLL => {
            const src_fq_hz = switch (bus.mux.main_pll.get()) {
                main_pll_src.HSE => hse_fq_hz,
                main_pll_src.HSI => hsi_fq_hz,
            };

            return bus.pll.main.getOutputHz(.P, src_fq_hz);
        },
    }

    return 0;
}
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

const PLL = struct {
    cfg: RCC.Reg,
    en_bit: FieldShiftType,
    rdy_bit: FieldShiftType,

    const Output = enum { P, Q, R };

    fn getOutputHz(self: *const PLL, out: Output, input_hz: BusType) BusType {
        const rdy = Field{ .reg = rcc.getReg(.CR), .width = 1, .shift = self.rdy_bit };
        if (rdy.isCleared())
            return 0;
        const cfg = rcc.getReg(self.cfg);
        const m = (Field{ .reg = cfg, .shift = 0, .width = 6 }).get();
        const n = (Field{ .reg = cfg, .shift = 6, .width = 9 }).get();

        return input_hz / m * n / switch (out) {
            .P => ((Field{ .reg = cfg, .shift = 16, .width = 4 }).get() + 1) * 2,
            .Q => (Field{ .reg = cfg, .shift = 24, .width = 4 }).get(),
            .R => (Field{ .reg = cfg, .shift = 28, .width = 3 }).get(),
        };
    }

    pub fn configure(self: *const PLL, comptime m: u6, comptime n: u9, comptime p: ?u4, comptime q: ?u4, comptime r: ?u3) void {
        comptime if (m < 2)
            @compileError("M divider value must be in range [2;63]");
        comptime if (n < 50 or n > 432)
            @compileError("N multiplier value must be in range [50;432]");
        comptime if (q != null and q.? < 2)
            @compileError("Q output divider value must be in range [2;15]");
        comptime if (r != null and r.? < 2)
            @compileError("R output divider value must be in range [2;7]");
        comptime if (p != null) {
            if ((p.? & 0x1) == 0x1)
                @compileError("P output divider value must be even");
            if (p.? < 2 or p.? > 8)
                @compileError("P output divider value must be in range[2;8]");
        };

        const en = Field{ .reg = rcc.getReg(.CR), .width = 1, .shift = self.en_bit };

        if (en.isAsserted())
            return;

        const cfg = rcc.getReg(self.cfg);
        (Field{ .reg = cfg, .shift = 0, .width = 6 }).set(m);
        (Field{ .reg = cfg, .shift = 6, .width = 9 }).set(n);
        if (p != null)
            (Field{ .reg = cfg, .shift = 16, .width = 4 }).set(p.? / 2 - 1);
        if (q != null)
            (Field{ .reg = cfg, .shift = 24, .width = 4 }).set(q.?);
        if (r != null)
            (Field{ .reg = cfg, .shift = 28, .width = 3 }).set(r.?);
        const rdy = Field{ .reg = rcc.getReg(.CR), .width = 1, .shift = self.rdy_bit };

        en.set(1);

        while (rdy.isCleared()) {}
    }
};

const FLASH = struct {
    port: BusType,
    fn getReg(self: *const FLASH, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const Reg = enum(BusType) {
        ACR = 0x00, // Flash access control register
        KEYR = 0x04, // Flash key register
        OPTKEYR = 0x08, // Flash option key register
        SR = 0x0C, // Flash status register
        CR = 0x10, // Flash control register
        OPTCR = 0x14, // Flash option control register
    };
    pub inline fn enableInstructionCache(self: *const FLASH) void {
        (Field{ .reg = self.getReg(.ACR), .shift = 9, .width = 1 }).set(1);
    }
    pub inline fn enableDataCache(self: *const FLASH) void {
        (Field{ .reg = self.getReg(.ACR), .shift = 10, .width = 1 }).set(1);
    }
    pub inline fn enablePrefetchBuffer(self: *const FLASH) void {
        (Field{ .reg = self.getReg(.ACR), .shift = 8, .width = 1 }).set(1);
    }
};

const DWT = struct {
    port: BusType,
    fn getReg(self: *const DWT, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    fn enableCycleCounter(self: *const DWT) void {
        scb.enableTrace();
        (Register{ .addr = self.getReg(.LAR) }).set(0xC5ACCE55);
        (Register{ .addr = self.getReg(.CYCCNT) }).reset();
        (Field{ .reg = self.getReg(.CTRL), .width = 1, .shift = 0 }).set(1);
    }
    const Reg = enum(BusType) {
        CTRL = 0x000, //  Control Register
        CYCCNT = 0x004, //  Cycle Count Register
        LAR = 0xFB0, // undocumented write only Lock Access Register
    };
};

const SCB = struct {
    port: BusType,
    fn getReg(self: *const SCB, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
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
        DHCSR = 0xDF0, //  Debug Halting Control and Status Register */
        DCRSR = 0xDF4, //  Debug Core Register Selector Register */
        DCRDR = 0xDF8, //  Debug Core Register Data Register */
        DEMCR = 0xDFC, //  Debug Exception and Monitor Control Register */
    };
    pub fn setSramVtor(self: *const SCB) void {
        (Field{ .reg = self.getReg(.VTOR), .width = 1, .shift = 29 }).set(1);
    }
    inline fn enableTrace(self: *const SCB) void {
        (Field{ .reg = self.getReg(.DEMCR), .width = 1, .shift = 24 }).set(1);
    }
    fn disableTrace(self: *const SCB) void {
        (Field{ .reg = self.getReg(.DEMCR), .width = 1, .shift = 24 }).reset();
    }
};

const MUXER = struct {
    src: FieldDesc,
    values: @TypeOf(enum {}),
    const FieldDesc = struct {
        reg: RCC.Reg,
        shift: FieldShiftType,
        width: FieldWidthType = 1,
    };
    pub fn set(self: *const MUXER, value: self.values) void {
        (Field{ .reg = rcc.getReg(self.src.reg), .shift = self.src.shift, .width = self.src.width }).set(@intFromEnum(value));
    }
    fn get(self: *const MUXER) self.values {
        const field: Field = Field{ .reg = rcc.getReg(self.src.reg), .shift = self.src.shift, .width = self.src.width };
        return @enumFromInt(field.get());
    }
};

const RCC = struct {
    port: BusType,
    fn getReg(self: *const RCC, reg: Reg) BusType {
        return self.port + @intFromEnum(reg);
    }
    const ClockSource = enum { HSI, HSE };
    const PeripherySwitch = struct {
        en_reg: Reg,
        shift: FieldShiftType,
    };

    const HSEMode = enum { Crystal };

    pub fn enableHSE(self: *const RCC, mode: HSEMode, fq: u32) void {
        hse_fq_hz = fq;

        switch (mode) {
            .Crystal => {
                const cr = self.getReg(.CR);
                const hseon: Field = .{ .reg = cr, .shift = 16, .width = 1 };
                const hserdy: Field = .{ .reg = cr, .rw = .ReadOnly, .shift = 17, .width = 1 };
                const hsebyp: Field = .{ .reg = cr, .shift = 18, .width = 1 };
                while (hserdy.isAsserted()) {}
                hsebyp.set(0);
                hseon.set(1);
                hseon.set(1);
                while (hserdy.isCleared()) {}
            },
        }
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
