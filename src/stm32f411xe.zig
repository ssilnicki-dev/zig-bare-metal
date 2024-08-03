const usbp = @import("usbp.zig");
const platform = @import("platform32.zig");
const SCS = @import("scs.zig").SCS;
const SPI = @import("spi.zig").SPI;

const BusType = platform.BusType;
const FieldShiftType = platform.FieldShiftType;
const FieldWidthType = platform.FieldWidthType;
const Field = platform.Field;

const GPIO = @import("gpio.zig").GPIO;

const hsi_fq_hz: u32 = 16000000;
var hse_fq_hz: u32 = undefined;

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
    fn ptr(self: *Register) *BusType {
        return @ptrFromInt(self.addr);
    }
};

const RxFifo = struct {
    reg: Register,
    cache: BusType = 0,
    bytes_read: usize = 0,

    fn read(self: *RxFifo) u8 {
        if (self.bytes_read % 4 == 0)
            self.cache = self.reg.get();
        defer self.bytes_read = (self.bytes_read + 1) % 4;
        const ret = @as([*]const u8, @ptrCast(&self.cache))[self.bytes_read];
        return ret;
    }
};

const TxFifo = struct {
    const Error = error{FifoIsFull};
    reg: Register,
    cache: BusType = 0,
    bytes_written: usize = 0,
    words_available_field: Field,

    fn write(self: *TxFifo, byte: u8) Error!void {
        if (self.bytes_written % 4 == 0 and self.words_available_field.get() == 0)
            return Error.FifoIsFull;
        @as([*]u8, @ptrCast(&self.cache))[self.bytes_written] = byte;
        self.bytes_written = (self.bytes_written + 1) % 4;
        if (self.bytes_written % 4 == 0) {
            self.reg.set(self.cache);
            self.cache = 0;
        }
    }
    fn flush(self: *TxFifo) void {
        if (self.bytes_written != 0 and self.words_available_field.get() > 0) {
            self.reg.set(self.cache);
        }
        self.cache = 0;
    }
};

const hal: struct {
    core: struct {
        const base: BusType = 0xE0000000;
        dwt: DWT = .{ .port = 0x1000 + base },
        scs: SCS = .{ .port = 0xE000 + base },
        nvic: NVIC = .{ .port = 0xE100 + base },
    } = .{},
    apb1: struct {
        const base: BusType = 0x40000000;
        power: POWER = .{ .port = 0x7000 + base },
    } = .{},
    apb2: struct {
        const base: BusType = 0x40010000;
        spi1: SPI = .{ .port = 0x3000 + base },
    } = .{},
    ahb1: struct {
        const base: BusType = 0x40020000;
        rcc: RCC = .{ .port = 0x3800 + base },
        flash: FLASH = .{ .port = 0x3C00 + base },
        gpioa: GPIO = .{ .port = 0x0000 + base },
        gpiob: GPIO = .{ .port = 0x0400 + base },
        gpioc: GPIO = .{ .port = 0x0800 + base },
        gpiod: GPIO = .{ .port = 0x0C00 + base },
        gpioe: GPIO = .{ .port = 0x1000 + base },
        gpioh: GPIO = .{ .port = 0x1C00 + base },
    } = .{},
    ahb2: struct {
        const base: BusType = 0x50000000;
        var usb_out_eps: [3]USB.OutEP = undefined;
        var usb_in_eps: [3]USB.InEP = undefined;
        usb: USB = .{
            .port = 0x0 + base,
            .out_ep = &usb_out_eps,
            .in_ep = &usb_in_eps,
        },
    } = .{},
    mux: struct {
        main_pll: MUXER = .{ .src = .{ .reg = .PLLCFGR, .shift = 22 }, .values = enum(u1) { HSI = 0, HSE = 1 } },
        sys_clock: MUXER = .{ .src = .{ .reg = .CFGR, .shift = 0, .width = 2 }, .rdy = .{ .reg = .CFGR, .shift = 2, .width = 2 }, .values = enum(u2) { HSI = 0, HSE = 1, PLL = 2 } },
    } = .{},
    pll: struct {
        main: PLL = .{ .cfg = .PLLCFGR, .en_bit = 24, .rdy_bit = 25, .src = enum(u1) { HSI = 0, HSE = 1 } },
    } = .{},
    presc: struct {
        ahb: PRESCALER = .{ .cfg = .CFGR, .value_type = .MostBit, .width = 4, .shift = 4 },
        apb1: PRESCALER = .{ .cfg = .CFGR, .value_type = .MostBit, .width = 3, .shift = 10 },
        apb2: PRESCALER = .{ .cfg = .CFGR, .value_type = .MostBit, .width = 3, .shift = 13 },
    } = .{},
} = .{};

pub const gpioa = &hal.ahb1.gpioa;
pub const gpiob = &hal.ahb1.gpiob;
pub const gpioc = &hal.ahb1.gpioc;
pub const gpiod = &hal.ahb1.gpiod;
pub const gpioe = &hal.ahb1.gpioe;
pub const gpioh = &hal.ahb1.gpioh;
pub const rcc = &hal.ahb1.rcc;
pub const scs = &hal.core.scs;
pub const nvic = &hal.core.nvic;
pub const flash = &hal.ahb1.flash;
pub const mux = &hal.mux;
pub const pll = &hal.pll;
pub const presc = &hal.presc;
pub const usb = &hal.ahb2.usb;
pub const power = &hal.apb1.power;
pub const spi1 = &hal.apb2.spi1;
pub const pin = GPIO.Pin;
pub const nvic_interrupt = NVIC.Interrupt;
pub const periphery = PERIPHERY{};

pub fn udelay(us: BusType, sys_clock_hz: BusType) void {
    const cyccnt: *volatile BusType = @ptrFromInt(0xE0001004); // see DWT
    cyccnt.* = 0;

    const wasted_cycles = sys_clock_hz / 1_000_000 * us;
    while (wasted_cycles > cyccnt.*) {}
}
pub fn enableCycleCounter() !void {
    try hal.core.dwt.enableCycleCounter();
}
pub fn getSysClockHz() BusType {
    const sysclock_src = hal.mux.sys_clock.values;
    const main_pll_src = hal.mux.main_pll.values;

    switch (hal.mux.sys_clock.get()) {
        sysclock_src.HSE => {
            return hse_fq_hz;
        },
        sysclock_src.HSI => {
            return hsi_fq_hz;
        },
        sysclock_src.PLL => {
            const src_fq_hz = switch (hal.mux.main_pll.get()) {
                main_pll_src.HSE => hse_fq_hz,
                main_pll_src.HSI => hsi_fq_hz,
            };

            return hal.pll.main.getOutputHz(.P, src_fq_hz);
        },
    }

    return 0;
}

fn CommonInterface(comptime T: type) type {
    return struct {
        fn getReg(self: *const T, reg: T.Reg) BusType {
            return switch (T) {
                inline USB.OutEP, USB.InEP => self.usb.port + @intFromEnum(reg) + @as(BusType, self.idx) * 0x20,
                inline else => self.port + @intFromEnum(reg),
            };
        }
    };
}
pub const USB = struct {
    port: BusType,
    out_ep: []OutEP,
    in_ep: []InEP,
    device: DeviceRoutines = .{},

    const EndpointDirection = usbp.UsbDevice.Endpoint.Direction;
    const EndpointType = usbp.UsbDevice.Endpoint.Type;

    usingnamespace CommonInterface(@This());

    pub const Interrupt = enum(FieldWidthType) {
        ModeMismatch = 1, // MMISM
        OTG = 2, // OTGINT
        SOF = 3, // SOFM
        RxFifoNonEmpty = 4, // RXFLVLM
        GlobalInNAK = 6, // GINAKEFFM
        GlobalOutNAK = 7, // GONAKEFFM
        EarlySuspend = 10, // ESUSPM
        UsbSuspend = 11, // USBSUSPM
        UsbReset = 12, // USBRST
        EnumerationDone = 13, // ENUMDNEM
        IsocOutPacketDropped = 14, // ISOODRPM
        EndOfPeriodicFrame = 15, // EOPFM
        InEndpoint = 18, // IEPINT
        OutEndpoint = 19, // OEPINT
        IncompleteIsocInTransfer = 20, // IISOIXFRM
        IncompleteIsocOutTransfer = 21, // IISOIXFRM
        ConnectorIdStatusChange = 28, // CIDSCHGM
        DisconnectDetected = 29, // DISCINT
        SessionRequest = 30, // SRQIM
        WakeUpDetected = 31, // WUIM
    };

    fn __clearRemoteWakeUp(self: *const USB) void {
        (Field{ .reg = self.getReg(.DCTL), .width = 1, .rw = .ReadWrite, .shift = 0 }).set(0);
    }

    fn isPendingInterrupt(irq: Interrupt, flags: BusType) bool {
        return flags & (@as(BusType, 1) << @intFromEnum(irq)) != 0;
    }

    fn __getActiveOutEp(self: *const USB) u4 {
        return @truncate((Field{ .reg = self.getReg(.GRXSTSR), .width = 4, .rw = .ReadOnly, .shift = 0 }).get());
    }

    const InEP linksection(".data") = struct {
        usb: *const USB = undefined,
        idx: usize = undefined,
        buffer: [2048]u8 = undefined,
        data: []const u8 = undefined,
        usingnamespace CommonInterface(@This());
        fn init(self: *const InEP) void {
            (Register{ .addr = self.getReg(.DIEPINT) }).set(0x287B);
            (Field{ .reg = self.getReg(.DIEPCTL), .width = 1, .rw = .ReadWrite, .shift = 21 }).set(0); // STALL
        }
        fn reset(self: *const InEP) void {
            const diepctl = self.getReg(.DIEPCTL);
            if ((Field{ .reg = diepctl, .width = 1, .rw = .ReadOnly, .shift = 31 }).isAsserted()) {
                if (self.idx != 0) {
                    (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 30 }).set(1);
                }
                (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 27 }).set(1);
            } else {
                (Register{ .addr = diepctl }).reset();
            }
            (Register{ .addr = self.getReg(.DIEPTSIZ) }).reset();
            (Register{ .addr = self.getReg(.DIEPINT) }).set(0xFB7F);
        }
        const Reg = enum(BusType) {
            DIEPCTL = 0x900, // OTG_FS device control IN endpoint control register
            DIEPINT = 0x908, // OTG_FS device endpoint-x interrupt register
            DIEPTSIZ = 0x910, // OTG_FS device IN endpoint transfer size register
            DTXFSTS = 0x918, // OTG_FS device IN endpoint transmit FIFO status register
        };

        const Interrupt = enum(FieldWidthType) {
            XFRCM = 0,
            EPDISD = 1,
            TOC = 3,
            ITTXFE = 4,
            INEPNM = 5,
            INEPNE = 6,
            TXFE = 7,
            PKTDRPSTS = 11,
            NAK = 13,
        };
        fn clearInterrupt(self: *const InEP, irq: InEP.Interrupt) void {
            (Field{ .reg = self.getReg(.DIEPINT), .width = 1, .rw = .WriteOnly, .shift = @intFromEnum(irq) }).set(1);
        }

        fn isPendingInterrupt(irq: InEP.Interrupt, flags: BusType) bool {
            return flags & (@as(BusType, 1) << @intFromEnum(irq)) != 0;
        }

        fn getInterrupts(self: *const InEP) BusType {
            return (Register{ .addr = self.getReg(.DIEPINT) }).get() &
                ((Register{ .addr = self.usb.getReg(.DIEPMSK) }).get() |
                (Field{ .reg = self.usb.getReg(.DIEPEMPMSK), .width = 1, .rw = .ReadOnly, .shift = @truncate(self.idx) }).get() << 7);
        }
        fn maskInterrupts(self: *const InEP) void {
            (Field{ .reg = self.usb.getReg(.DAINTMSK), .width = 1, .rw = .ReadWrite, .shift = @truncate(self.idx) }).set(0);
        }
        fn unmaskInterrupts(self: *const InEP) void {
            (Field{ .reg = self.usb.getReg(.DAINTMSK), .width = 1, .rw = .ReadWrite, .shift = @truncate(self.idx) }).set(1);
        }
        fn enableInterrupts(self: *const USB, in_irqs: []const InEP.Interrupt) void {
            const diepmsk = Register{ .addr = self.getReg(.DIEPMSK) };
            var value = diepmsk.get();
            for (in_irqs) |irq|
                value |= @as(BusType, 1) << @intFromEnum(irq);
            diepmsk.set(value);
        }
        fn setMaxPacketSize(self: *const InEP, size: usize) void {
            USB.setMaxPacketSize(self.getReg(.DIEPCTL), size, self.idx);
        }
        fn getMaxPacketSize(self: *const InEP) usize {
            return USB.getMaxPacketSize(self.getReg(.DIEPCTL), self.idx);
        }
        fn activate(self: *const InEP, packet_size: u11, ep_type: EndpointType) void {
            const diepctl = self.getReg(.DIEPCTL);
            self.unmaskInterrupts();
            self.setMaxPacketSize(packet_size);
            if (ep_type != .Control or self.idx != 0) {
                (Field{ .reg = diepctl, .width = 2, .rw = .ReadWrite, .shift = 18 }).set(@intFromEnum(ep_type));
            }
            (Field{ .reg = diepctl, .width = 4, .rw = .ReadWrite, .shift = 22 }).set(self.idx); // TXFNUM
            (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 28 }).set(1); // Undocumentd!!!!
            (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 15 }).set(1); // USBAEP
        }

        fn getEpType(self: *const InEP) EndpointType {
            return @enumFromInt((Field{ .reg = self.getReg(.DIEPCTL), .width = 2, .rw = .ReadOnly, .shift = 18 }).get());
        }

        fn sendStatus(self: *InEP) void {
            self.startTx(self.buffer[0..0]);
        }
        fn continueTx(self: *InEP) void {
            self.startTx(self.data);
        }
        fn stall(self: *InEP) void {
            const diepctl = self.getReg(.DIEPCTL);
            const epena = Field{ .reg = diepctl, .width = 1, .rw = .ReadOnly, .shift = 31 };
            const epdis = Field{ .reg = diepctl, .width = 1, .rw = .ReadOnly, .shift = 30 };
            if (self.idx != 0 and epena.isCleared())
                epdis.set(1);
            (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 21 }).set(1); // STALL
        }

        fn startTx(self: *InEP, data: []const u8) void {
            self.data = data;
            const packet_size = self.getMaxPacketSize();
            const pkts = switch (self.idx) {
                0 => 1,
                else => (data.len + packet_size - 1) / packet_size,
            };
            const tx_size = switch (self.idx) {
                0 => @min(packet_size, data.len),
                else => data.len,
            };

            const dieptsiz = self.getReg(.DIEPTSIZ);
            (Field{ .reg = dieptsiz, .width = switch (self.idx) {
                0 => 2,
                else => 11,
            }, .rw = .ReadWrite, .shift = 19 }).set(pkts);
            (Field{ .reg = dieptsiz, .width = switch (self.idx) {
                0 => 7,
                else => 19,
            }, .rw = .ReadWrite, .shift = 0 }).set(tx_size);

            const ep_type = self.getEpType();
            if (ep_type == .Isochronous) {
                (Field{ .reg = dieptsiz, .width = 2, .rw = .ReadWrite, .shift = 29 }).set(1); // Undocumented
            }

            const diepctl = self.getReg(.DIEPCTL);
            (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 26 }).set(1); // CNAK
            (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 31 }).set(1); // EPENA

            if (ep_type != .Isochronous) {
                if (tx_size > 0) {
                    self.usb.enableTxFifoEmptyInterrupt(self);
                }
            } else {
                if ((Field{ .reg = self.usb.getReg(.DSTS), .width = 1, .rw = .ReadOnly, .shift = 8 }).isCleared()) {
                    (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 29 }).set(1); // SODDFRM -> odd frame
                } else {
                    (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 28 }).set(1); // SEVNFRM -> even frame
                }
                self.write();
            }
        }
        fn write(self: *InEP) void {
            var tx_fifo = TxFifo{
                .reg = self.usb.getFifoRegister(self.idx),
                .words_available_field = Field{ .reg = self.getReg(.DTXFSTS), .width = 16, .rw = .ReadOnly, .shift = 0 },
            };
            const write_size = @min(self.getMaxPacketSize(), self.data.len);
            for (0..write_size) |_| {
                tx_fifo.write(self.data[0]) catch {
                    break;
                };
                self.data = self.data[1..];
            }
            tx_fifo.flush();
            if (self.data.len == 0)
                self.usb.disableTxFifoEmptyInterrupt(self);
        }
    };

    fn enableTxFifoEmptyInterrupt(self: *const USB, ep: *const InEP) void {
        (Field{ .reg = self.getReg(.DIEPEMPMSK), .width = 1, .rw = .ReadWrite, .shift = @truncate(ep.idx) }).set(1);
    }
    fn disableTxFifoEmptyInterrupt(self: *const USB, ep: *const InEP) void {
        (Field{ .reg = self.getReg(.DIEPEMPMSK), .width = 1, .rw = .ReadWrite, .shift = @truncate(ep.idx) }).set(0);
    }

    fn setMaxPacketSize(reg: BusType, size: usize, ep_idx: usize) void {
        const value: usize = switch (ep_idx) {
            0 => switch (size) {
                0...8 => 3,
                9...16 => 2,
                17...32 => 1,
                else => 0, // 64
            },
            else => size,
        };
        const width: FieldWidthType = switch (ep_idx) {
            0 => 2,
            else => 11,
        };
        (Field{ .reg = reg, .width = width, .rw = .ReadWrite, .shift = 0 }).set(value);
    }

    fn getMaxPacketSize(reg: BusType, ep_idx: usize) usize {
        const width: FieldWidthType = switch (ep_idx) {
            0 => 2,
            else => 11,
        };
        const size = (Field{ .reg = reg, .width = width, .rw = .ReadWrite, .shift = 0 }).get();
        return switch (ep_idx) {
            0 => switch (@as(u2, @truncate(size))) {
                3 => 8,
                2 => 16,
                1 => 32,
                0 => 64,
            },
            else => size,
        };
    }

    fn getFifoRegister(self: *const USB, ep: usize) Register {
        return Register{ .addr = self.getReg(.DFIFO_BASE) + @as(BusType, ep) * 0x1000 };
    }

    const OutEP linksection(".data") = struct {
        usb: *const USB = undefined,
        idx: usize = undefined,
        buffer: [2048]u8 align(4) = undefined,
        data: []align(4) u8 = undefined,
        usingnamespace CommonInterface(@This());

        fn init(self: *const OutEP) void {
            (Register{ .addr = self.getReg(.DOEPINT) }).set(0xFB7F);
            (Field{ .reg = self.getReg(.DOEPCTL), .width = 1, .rw = .ReadWrite, .shift = 21 }).set(0); // STALL
            (Field{ .reg = self.getReg(.DOEPCTL), .width = 1, .rw = .ReadWrite, .shift = 27 }).set(1); // SNACK
        }
        fn initRM(self: *const OutEP) void {
            (Field{ .reg = self.getReg(.DOEPCTL), .width = 1, .rw = .ReadWrite, .shift = 27 }).set(1); // SNACK
        }
        fn reset(self: *const OutEP) void {
            const doepctl = self.getReg(.DOEPCTL);
            if ((Field{ .reg = doepctl, .width = 1, .rw = .ReadOnly, .shift = 31 }).isAsserted()) {
                if (self.idx != 0) {
                    (Field{ .reg = doepctl, .width = 1, .rw = .ReadWrite, .shift = 30 }).set(1);
                }
                (Field{ .reg = doepctl, .width = 1, .rw = .ReadWrite, .shift = 27 }).set(1);
            } else {
                (Register{ .addr = doepctl }).reset();
            }
            (Register{ .addr = self.getReg(.DOEPTSIZ) }).reset();
            (Register{ .addr = self.getReg(.DOEPINT) }).set(0xFB7F);
        }
        fn maskInterrupts(self: *const OutEP) void {
            (Field{ .reg = self.usb.getReg(.DAINTMSK), .width = 1, .rw = .ReadWrite, .shift = 16 + @as(FieldShiftType, @truncate(self.idx)) }).set(0);
        }
        fn unmaskInterrupts(self: *const OutEP) void {
            (Field{ .reg = self.usb.getReg(.DAINTMSK), .width = 1, .rw = .ReadWrite, .shift = 16 + @as(FieldShiftType, @truncate(self.idx)) }).set(1);
        }
        fn enableInterrupts(self: *const USB, out_irqs: []const OutEP.Interrupt) void {
            const doepmsk = Register{ .addr = self.getReg(.DOEPMSK) };
            var value = doepmsk.get();
            for (out_irqs) |irq|
                value |= @as(BusType, 1) << @intFromEnum(irq);
            doepmsk.set(value);
            value = doepmsk.get();
        }
        fn setMaxPacketSize(self: *const OutEP, size: usize) void {
            USB.setMaxPacketSize(self.getReg(.DOEPCTL), size, self.idx);
        }
        const Reg = enum(BusType) {
            DOEPCTL = 0xB00, // OTG_FS device control OUT endpoint control register
            DOEPINT = 0xB08, // OTG_FS device endpoint-x interrupt register
            DOEPTSIZ = 0xB10, // OTG_FS device OUT endpoint transfer size register
        };
        const Interrupt = enum(FieldWidthType) {
            XFRCM = 0,
            EPDM = 1,
            STUPM = 3,
            OTEPDM = 4,
            STSPHSRXM = 5,
            OUTPKTERRM = 8,
            BERRM = 12,
            NAK = 13,
        };

        fn getInterrupts(self: *const OutEP) BusType {
            return (Register{ .addr = self.getReg(.DOEPINT) }).get() & (Register{ .addr = self.usb.getReg(.DOEPMSK) }).get();
        }

        fn isPendingInterrupt(irq: OutEP.Interrupt, flags: BusType) bool {
            return flags & (@as(BusType, 1) << @intFromEnum(irq)) != 0;
        }

        fn clearInterrupt(self: *const OutEP, irq: OutEP.Interrupt) void {
            (Field{ .reg = self.getReg(.DOEPINT), .width = 1, .rw = .WriteOnly, .shift = @intFromEnum(irq) }).set(1);
        }

        fn read(self: *OutEP, size: usize) void {
            var rx_fifo = RxFifo{ .reg = self.usb.getFifoRegister(self.idx) };
            if (self.idx == 0)
                self.data = self.buffer[0..0];
            const rx_size = @min(size, self.buffer.len - self.data.len);
            const buf = self.buffer[self.data.len..(self.data.len + rx_size)];
            for (buf) |*b|
                b.* = rx_fifo.read();
            self.popData(size - rx_size);
            self.data = self.buffer[0..(self.data.len + rx_size)];
        }
        fn popData(self: *const OutEP, size: usize) void {
            var fifo = RxFifo{ .reg = self.usb.getFifoRegister(self.idx) };
            for (0..size) |_|
                _ = fifo.read();
        }
        fn startRx(self: *const OutEP) void {
            const diepctl = self.getReg(.DOEPCTL);
            (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 26 }).set(1); // CNAK
            (Field{ .reg = diepctl, .width = 1, .rw = .ReadWrite, .shift = 31 }).set(1); // EPENA
        }

        fn prepareRx(self: *OutEP, size: u19, cntr: u10) void {
            const doeptsiz = self.getReg(.DOEPTSIZ);
            (Register{ .addr = doeptsiz }).reset();
            (Field{ .reg = doeptsiz, .width = 19, .rw = .ReadWrite, .shift = 0 }).set(size);
            (Field{ .reg = doeptsiz, .width = 10, .rw = .ReadWrite, .shift = 19 }).set(cntr);
            self.data = self.buffer[0..0];
        }
        fn prepareSetupRx(self: *OutEP) void {
            if (self.idx != 0)
                return;
            const doeptsiz = self.getReg(.DOEPTSIZ);
            (Register{ .addr = doeptsiz }).reset();
            (Field{ .reg = doeptsiz, .width = 7, .rw = .ReadWrite, .shift = 0 }).set(8 * 3);
            (Field{ .reg = doeptsiz, .width = 2, .rw = .ReadWrite, .shift = 29 }).set(3);
            (Field{ .reg = doeptsiz, .width = 1, .rw = .ReadWrite, .shift = 19 }).set(1);
            self.data = self.buffer[0..0];
        }
        fn activate(self: *const OutEP, packet_size: u11, ep_type: EndpointType) void {
            const doepctl = self.getReg(.DOEPCTL);
            self.unmaskInterrupts();
            self.setMaxPacketSize(packet_size);
            if (self.idx != 0) {
                (Field{ .reg = doepctl, .width = 2, .rw = .ReadWrite, .shift = 18 }).set(@intFromEnum(ep_type));
            }
            (Field{ .reg = doepctl, .width = 1, .rw = .ReadWrite, .shift = 28 }).set(1); // Undocumentd!!!!
            (Field{ .reg = doepctl, .width = 1, .rw = .ReadWrite, .shift = 15 }).set(1); // USBAEP
        }
    };

    fn __resetEndPoints(self: *const USB) void {
        for (self.in_ep[0..]) |*ep| {
            ep.reset();
        }
        for (self.out_ep[0..]) |*ep| {
            ep.reset();
        }
    }
    fn __initEndPoints(self: *const USB) void {
        for (self.in_ep[0..]) |*ep| {
            ep.init();
        }
        for (self.out_ep[0..]) |*ep| {
            ep.init();
        }
    }
    fn __maskEndpointInterrupt(self: *const USB, ep: usize, dir: EndpointDirection) void {
        switch (dir) {
            .In => self.getInEP(ep).maskInterrupts(),
            .Out => self.getOutEP(ep).maskInterrupts(),
        }
    }
    fn enableEndpointInterrupts(self: *const USB, in_irqs: []const InEP.Interrupt, out_irqs: []const OutEP.Interrupt) void {
        if (in_irqs.len > 0)
            InEP.enableInterrupts(self, in_irqs);
        if (out_irqs.len > 0)
            OutEP.enableInterrupts(self, out_irqs);
    }

    fn setDeviceAddress(self: *const USB, addr: u7) void {
        (Field{ .reg = self.getReg(.DCFG), .width = 7, .rw = .ReadWrite, .shift = 4 }).set(addr);
    }

    fn activateSetup(self: *const USB, ep0size: usize) void {
        self.in_ep[0].setMaxPacketSize(ep0size);
        (Field{ .reg = self.getReg(.DCTL), .width = 1, .rw = .ReadWrite, .shift = 8 }).set(1); // CGINAK
    }

    const Speed = enum(u2) { FullSpeed = 0b11 };
    fn setTimeoutCalibration(self: *const USB, value: u3) void {
        (Field{ .reg = self.getReg(.GUSBCFG), .width = 3, .rw = .ReadWrite, .shift = 0 }).set(value);
    }

    fn setSpeed(self: *const USB, speed: Speed) void {
        const value: u4 = switch (speed) {
            .FullSpeed => switch (getSysClockHz()) {
                0...14_999_999 => 0xF,
                15_000_000...15_999_999 => 0xE,
                16_000_000...17_199_999 => 0xD,
                17_200_000...18_499_999 => 0xC,
                18_500_000...19_999_999 => 0xB,
                20_000_000...21_799_999 => 0xA,
                21_800_000...23_999_999 => 0x9,
                24_000_000...27_699_999 => 0x8,
                27_700_000...31_999_999 => 0x7,
                else => 0x6,
            },
        };
        (Field{ .reg = self.getReg(.GUSBCFG), .width = 4, .rw = .ReadWrite, .shift = 10 }).set(value);
        (Field{ .reg = self.getReg(.DCFG), .width = 2, .rw = .ReadWrite, .shift = 0 }).set(@intFromEnum(speed));
    }

    const Mode = enum(u1) {
        Device = 0,
    };

    const FifoConfig = struct {
        rxFifoSize: u16,
        txFifoSize: []const u16,
    };

    fn __getSpeed(self: *const USB) u2 {
        return @truncate((Field{ .reg = self.getReg(.DSTS), .width = 2, .rw = .ReadOnly, .shift = 1 }).get());
    }
    fn enableVBusSensing(self: *const USB) void {
        (Field{ .reg = self.getReg(.GCCFG), .width = 1, .rw = .ReadWrite, .shift = 19 }).set(1);
        (Field{ .reg = self.getReg(.GCCFG), .width = 1, .rw = .ReadWrite, .shift = 21 }).set(0);
    }
    fn disableVBusSensing(self: *const USB) void {
        (Field{ .reg = self.getReg(.DCTL), .width = 1, .rw = .ReadWrite, .shift = 1 }).set(1);
        (Field{ .reg = self.getReg(.GCCFG), .width = 1, .rw = .ReadWrite, .shift = 21 }).set(1);
        (Field{ .reg = self.getReg(.GCCFG), .width = 1, .rw = .ReadWrite, .shift = 19 }).set(0);
        (Field{ .reg = self.getReg(.GCCFG), .width = 1, .rw = .ReadWrite, .shift = 18 }).set(0);
    }
    fn disconnect(self: *const USB) void {
        self.powerUp();
        (Field{ .reg = self.getReg(.DCTL), .width = 1, .rw = .ReadWrite, .shift = 1 }).set(1);
    }
    pub fn connect(self: *const USB) void {
        self.activateTransceiver();
        self.powerUp();
        (Field{ .reg = self.getReg(.DCTL), .width = 1, .rw = .ReadWrite, .shift = 1 }).set(0);
        self.enableInterrupts();
    }

    fn configureFifo(self: *const USB, config: FifoConfig) void {
        (Field{ .reg = self.getReg(.GRXFSIZ), .width = 16, .rw = .ReadWrite, .shift = 0 }).set(config.rxFifoSize);
        var offset = config.rxFifoSize;

        if (config.txFifoSize.len == 0)
            return;
        var dieptx_offset = Field{ .reg = self.getReg(.DIEPTXF0), .width = 16, .rw = .ReadWrite, .shift = 0 };
        var dieptx_size = Field{ .reg = self.getReg(.DIEPTXF0), .width = 16, .rw = .ReadWrite, .shift = 16 };
        dieptx_offset.set(offset);
        dieptx_size.set(config.txFifoSize[0]);
        offset += config.txFifoSize[0];

        dieptx_offset.reg = self.getReg(.DIEPTXF1);
        dieptx_size.reg = self.getReg(.DIEPTXF1);
        for (1..config.txFifoSize.len) |idx| {
            dieptx_offset.set(offset);
            dieptx_size.set(config.txFifoSize[idx]);
            offset += config.txFifoSize[idx];
            dieptx_offset.reg += 0x20;
            dieptx_size.reg += 0x20;
        }
    }

    fn powerDown(self: *const USB) void {
        (Field{ .reg = self.getReg(.PCGCCTL), .width = 1, .rw = .ReadWrite, .shift = 0 }).set(1);
        (Field{ .reg = self.getReg(.PCGCCTL), .width = 1, .rw = .ReadWrite, .shift = 1 }).set(1);
    }
    fn powerUp(self: *const USB) void {
        (Field{ .reg = self.getReg(.PCGCCTL), .width = 1, .rw = .ReadWrite, .shift = 1 }).set(0);
        (Field{ .reg = self.getReg(.PCGCCTL), .width = 1, .rw = .ReadWrite, .shift = 0 }).set(0);
    }

    fn __maskAllInterrupts(self: *const USB) void {
        (Register{ .addr = self.getReg(.GINTMSK) }).reset();
    }
    fn maskInterrupt(self: *const USB, irq: Interrupt) void {
        (Field{ .reg = self.getReg(.GINTMSK), .width = 1, .rw = .ReadWrite, .shift = @intFromEnum(irq) }).set(0);
    }
    fn unmaskInterrupt(self: *const USB, irq: Interrupt) void {
        (Field{ .reg = self.getReg(.GINTMSK), .width = 1, .rw = .ReadWrite, .shift = @intFromEnum(irq) }).set(1);
    }

    const CapabilityValue = enum(u1) {
        Incapable = 0,
        Capable = 1,
    };
    const CapabilityType = enum {
        HostNegotiationProtocol,
        SessionRequestProtocol,
    };

    fn setCapability(self: *const USB, cap_t: CapabilityType, cap_v: CapabilityValue) void {
        (Field{ .reg = self.getReg(.GUSBCFG), .width = 1, .rw = .ReadWrite, .shift = switch (cap_t) {
            .HostNegotiationProtocol => 9,
            .SessionRequestProtocol => 8,
        } }).set(@intFromEnum(cap_v));
    }

    fn __clearInterrupts(self: *const USB) void {
        (Register{ .addr = self.getReg(.GINTSTS) }).set(0xBFFFFFFF);
    }
    fn clearInterrupt(self: *const USB, irq: Interrupt) void {
        (Register{ .addr = self.getReg(.GINTSTS) }).set(@as(BusType, 1) << @intFromEnum(irq));
    }
    fn __clearPendingInterrupts(self: *const USB) void {
        (Register{ .addr = self.getReg(.DIEPMSK) }).reset();
        (Register{ .addr = self.getReg(.DOEPMSK) }).reset();
        (Register{ .addr = self.getReg(.DAINTMSK) }).reset();
    }
    fn disableInterrupts(self: *const USB) void {
        (Field{ .reg = self.getReg(.GAHBCFG), .rw = .ReadWrite, .width = 1, .shift = 0 }).set(0);
    }
    fn enableInterrupts(self: *const USB) void {
        (Field{ .reg = self.getReg(.GAHBCFG), .rw = .ReadWrite, .width = 1, .shift = 0 }).set(1);
    }

    fn flushTxFifo(self: *const USB, fifo: union(enum) { idx: u4, all: void }) void {
        const grstctl = self.getReg(.GRSTCTL);
        const ahbidl = Field{ .reg = grstctl, .width = 1, .rw = .ReadOnly, .shift = 31 };
        while (ahbidl.isCleared()) {}

        const value: u5 = switch (fifo) {
            .idx => |*idx| idx.*,
            .all => 0x10,
        };

        (Field{ .reg = grstctl, .width = 5, .rw = .ReadWrite, .shift = 6 }).set(value);
        const txfflsh = Field{ .reg = grstctl, .width = 1, .rw = .ReadWrite, .shift = 5 };
        txfflsh.set(1);
        while (txfflsh.isAsserted()) {}
    }
    fn flushRxFifo(self: *const USB) void {
        const grstctl = self.getReg(.GRSTCTL);
        const ahbidl = Field{ .reg = grstctl, .width = 1, .rw = .ReadOnly, .shift = 31 };
        while (ahbidl.isCleared()) {}

        const rxfflsh = Field{ .reg = grstctl, .width = 1, .rw = .ReadWrite, .shift = 4 };
        rxfflsh.set(1);
        while (rxfflsh.isAsserted()) {}
    }

    const Reg = enum(BusType) {
        GOTGINT = 0x04, // OTG_FS interrupt register (OTG_FS_GOTGINT)
        GAHBCFG = 0x08, // OTG_FS AHB configuration register
        GUSBCFG = 0x0C, // OTG_FS USB configuration register
        GRSTCTL = 0x10, // OTG_FS reset register
        GINTSTS = 0x14, // OTG_FS core interrupt register
        GINTMSK = 0x18, // OTG_FS interrupt mask register
        GRXSTSR = 0x1C, // OTG_FS Receive status debug read
        GRXSTSP = 0x20, // OTG_FS Receive status read and pop
        GRXFSIZ = 0x24, // OTG_FS Receive FIFO size register
        DIEPTXF0 = 0x28, // OTG_FS Endpoint 0 Transmit FIFO size
        GCCFG = 0x38, //  OTG_FS general core configuration register
        PCGCCTL = 0xE00, // OTG_FS power and clock gating control register
        DCFG = 0x800, // OTG_FS device configuration register
        DCTL = 0x804, // OTG_FS device control register
        DSTS = 0x808, // OTG_FS device status register
        DIEPMSK = 0x810, // OTG_FS device IN endpoint common interrupt mask register
        DOEPMSK = 0x814, // OTG_FS device OUT endpoint common interrupt mask register
        DAINT = 0x818, // OTG_FS device all endpoints interrupt register
        DAINTMSK = 0x81C, // OTG_FS all endpoints interrupt mask register
        DIEPEMPMSK = 0x834, // OTG_FS device IN endpoint FIFO empty interrupt mask register
        DIEPTXF1 = 0x104, // OTG_FS device IN endpoint transmit FIFO size register
        DIEPTXF2 = 0x108, // OTG_FS device IN endpoint transmit FIFO size register
        DIEPTXF3 = 0x10C, // OTG_FS device IN endpoint transmit FIFO size register
        DFIFO_BASE = 0x1000, // OTG_FS Data FIFO address space start
    };

    fn reset(self: *const USB) void {
        (Field{ .reg = self.getReg(.GUSBCFG), .rw = .ReadWrite, .width = 1, .shift = 6 }).set(1);
        const ahb_ilde = Field{ .reg = self.getReg(.GRSTCTL), .rw = .ReadWrite, .width = 1, .shift = 31 };
        while (ahb_ilde.isCleared()) {}
        const csrst = Field{ .reg = self.getReg(.GRSTCTL), .rw = .ReadWrite, .width = 1, .shift = 0 };
        csrst.set(1);
        while (csrst.isAsserted()) {}
    }
    fn activateTransceiver(self: *const USB) void {
        (Field{ .reg = self.getReg(.GCCFG), .rw = .ReadWrite, .width = 1, .shift = 16 }).set(1);
    }

    fn setMode(self: *const USB, mode: Mode) void {
        const reg = self.getReg(.GUSBCFG);
        const host = Field{ .reg = reg, .rw = .ReadWrite, .width = 1, .shift = 29 };
        const device = Field{ .reg = reg, .rw = .ReadWrite, .width = 1, .shift = 30 };
        host.set(0);
        device.set(0);
        switch (mode) {
            .Device => {
                device.set(1);
                udelay(10_000, getSysClockHz());
                while (device.isCleared()) {}
            },
        }
    }

    fn selectInternalPhy(self: *const USB) void {
        (Field{ .reg = self.getReg(.GUSBCFG), .width = 1, .rw = .ReadWrite, .shift = 6 }).set(1);
    }

    const PacketType = enum(u4) {
        Nak = 1,
        DataReceived = 2,
        TransferComplete = 3,
        SetupComplete = 4,
        SetupDataReceived = 6,
    };

    pub const DeviceRoutines = struct {
        fn encodeUnicode(input: u32, buf: []u8) void {
            for (buf, 0..) |*v, i| {
                if (i % 2 == 1) {
                    v.* = 0;
                    continue;
                }
                const hex: u4 = @truncate(input >> @truncate(28 - i * 2));
                if (hex < 0xA) {
                    v.* = @as(u8, hex) + '0';
                } else v.* = @as(u8, hex) + 'A' - 0xA;
            }
        }

        pub fn serialNumberGetter(buf: []u8) usbp.UsbProtoError![]const u8 {
            if (buf.len < 24) return usbp.UsbProtoError.OOM;

            var serial0 = (Register{ .addr = 0x1FFF7A10 }).get();
            const serial1 = (Register{ .addr = 0x1FFF7A14 }).get();
            serial0 += (Register{ .addr = 0x1FFF7A18 }).get();
            encodeUnicode(serial0, buf[0..16]);
            encodeUnicode(serial1, buf[16..24]);
            return buf[0..24];
        }

        pub const IrqHandlerContext linksection(".data") = struct {
            device: *const usbp.UsbDevice,
            setConfiguration: *const fn (u8, *IrqHandlerContext) usbp.UsbProtoError!void,
            currentConfiguration: ?*const usbp.UsbDevice.Configuration = null,
        };

        pub fn otgFsIrqHandler(self: *const DeviceRoutines, ctx: *IrqHandlerContext) void {
            const core: *const USB = @alignCast(@fieldParentPtr("device", self));
            const irqs = (Register{ .addr = core.getReg(.GINTSTS) }).get();
            if (isPendingInterrupt(.RxFifoNonEmpty, irqs)) {
                rxFifioIrqHandler(core);
            }
            if (isPendingInterrupt(.OutEndpoint, irqs)) {
                outIrqHandler(core, ctx);
            }
            if (isPendingInterrupt(.InEndpoint, irqs)) {
                inIrqHandler(core, ctx);
            }
            if (isPendingInterrupt(.UsbReset, irqs)) {
                usbResetIrqHandler(core);
            }
            if (isPendingInterrupt(.EnumerationDone, irqs)) {
                enumerationIrqDoneHandler(core, ctx);
            }
            if (isPendingInterrupt(.UsbSuspend, irqs)) {
                suspendIrqHandler(core);
            }
            if (isPendingInterrupt(.OTG, irqs)) {
                otgIrqHandler(core);
            }
            if (isPendingInterrupt(.WakeUpDetected, irqs)) {
                core.clearInterrupt(.WakeUpDetected);
            }
            if (isPendingInterrupt(.SOF, irqs)) {
                core.clearInterrupt(.SOF);
            }
            if (isPendingInterrupt(.GlobalOutNAK, irqs)) {
                core.clearInterrupt(.GlobalOutNAK);
            }
            if (isPendingInterrupt(.IncompleteIsocInTransfer, irqs)) {
                core.clearInterrupt(.IncompleteIsocInTransfer);
            }
            if (isPendingInterrupt(.IncompleteIsocOutTransfer, irqs)) {
                core.clearInterrupt(.IncompleteIsocOutTransfer);
            }
            if (isPendingInterrupt(.SessionRequest, irqs)) {
                core.clearInterrupt(.SessionRequest);
            }
            if (isPendingInterrupt(.ModeMismatch, irqs)) {
                core.clearInterrupt(.ModeMismatch);
            }
        }

        fn processSetupRequest(core: *const USB, req: usbp.SetupRequest, ctx: *IrqHandlerContext) void {
            switch (req.request) {
                .device => |dev_req| {
                    switch (dev_req) {
                        .GetStatus => {},
                        .GetConfiguration => {},
                        .SetConfiguration => {
                            ctx.setConfiguration(@truncate(req.value), ctx) catch {
                                core.in_ep[0].stall();
                                return;
                            };
                            core.in_ep[0].sendStatus();
                        },
                        .SetDescriptor => {},
                        .SetFeature => {},
                        .SetAddress => {
                            core.setDeviceAddress(@truncate(req.value));
                            core.in_ep[0].sendStatus();
                        },
                        .GetDescriptor => {
                            const descriptor = ctx.device.getDescriptor(req, core.in_ep[0].buffer[0..]) catch {
                                core.in_ep[0].stall();
                                return;
                            };
                            core.in_ep[0].startTx(descriptor);
                        },
                        else => {},
                    }
                },
                else => {},
            }
        }
        fn inIrqHandler(core: *const USB, ctx: *IrqHandlerContext) void {
            _ = ctx; // autofix
            const eps_irqs = (Field{ .reg = core.getReg(.DAINT), .width = 16, .rw = .ReadOnly, .shift = 0 }).get() & (Field{ .reg = core.getReg(.DAINTMSK), .width = 16, .rw = .ReadOnly, .shift = 0 }).get();
            var ep_irq = Field{ .reg = @intFromPtr(&eps_irqs), .width = 1, .rw = .ReadOnly, .shift = 0 };

            for (core.in_ep[0..]) |*ep| {
                ep_irq.shift = @truncate(ep.idx);
                if (ep_irq.isCleared())
                    continue;
                const ep_irqs = ep.getInterrupts();
                if (InEP.isPendingInterrupt(.TXFE, ep_irqs)) {
                    ep.write();
                }
                if (InEP.isPendingInterrupt(.XFRCM, ep_irqs)) {
                    defer ep.clearInterrupt(.XFRCM);
                    core.disableTxFifoEmptyInterrupt(ep);
                    if (ep.idx == 0) {
                        if (ep.data.len > 0) {
                            ep.continueTx();
                        } else {
                            core.out_ep[0].prepareSetupRx();
                            core.out_ep[0].startRx();
                        }
                    }
                }
                if (InEP.isPendingInterrupt(.TOC, ep_irqs)) {
                    defer ep.clearInterrupt(.TOC);
                }
                if (InEP.isPendingInterrupt(.ITTXFE, ep_irqs)) {
                    defer ep.clearInterrupt(.ITTXFE);
                }
                if (InEP.isPendingInterrupt(.INEPNE, ep_irqs)) {
                    defer ep.clearInterrupt(.INEPNE);
                }
                if (InEP.isPendingInterrupt(.EPDISD, ep_irqs)) {
                    defer ep.clearInterrupt(.EPDISD);
                    core.flushTxFifo(.{ .idx = @truncate(ep.idx) });
                }
            }
        }
        fn outIrqHandler(core: *const USB, ctx: *IrqHandlerContext) void {
            const eps_irqs = (Field{ .reg = core.getReg(.DAINT), .width = 16, .rw = .ReadOnly, .shift = 16 }).get() & (Field{ .reg = core.getReg(.DAINTMSK), .width = 16, .rw = .ReadOnly, .shift = 16 }).get();
            var ep_irq = Field{ .reg = @intFromPtr(&eps_irqs), .width = 1, .rw = .ReadOnly, .shift = 0 };

            for (core.out_ep[0..]) |*ep| {
                ep_irq.shift = @truncate(ep.idx);
                if (ep_irq.isCleared())
                    continue;
                const ep_irqs = ep.getInterrupts();
                if (OutEP.isPendingInterrupt(.STUPM, ep_irqs)) {
                    defer ep.clearInterrupt(.STUPM);
                    const req = usbp.getSetupRequest(ep.data) catch {
                        if (ep.idx == 0 and ep.data.len == 0) {
                            core.out_ep[0].prepareSetupRx();
                            core.out_ep[0].startRx();
                        }
                        return;
                    };
                    processSetupRequest(core, req, ctx);
                }
                if (OutEP.isPendingInterrupt(.XFRCM, ep_irqs)) {
                    defer ep.clearInterrupt(.XFRCM);
                    if (ep.idx == 0 and ep.data.len == 0) {
                        core.out_ep[0].prepareSetupRx();
                        core.out_ep[0].startRx();
                    }
                }
                if (OutEP.isPendingInterrupt(.EPDM, ep_irqs)) {
                    ep.clearInterrupt(.EPDM);
                }
                if (OutEP.isPendingInterrupt(.BERRM, ep_irqs)) {
                    ep.clearInterrupt(.BERRM);
                }
                if (OutEP.isPendingInterrupt(.OTEPDM, ep_irqs)) {
                    ep.clearInterrupt(.OTEPDM);
                }
                if (OutEP.isPendingInterrupt(.NAK, ep_irqs)) {
                    ep.clearInterrupt(.NAK);
                }
                if (OutEP.isPendingInterrupt(.OUTPKTERRM, ep_irqs)) {
                    ep.clearInterrupt(.OUTPKTERRM);
                }
                if (OutEP.isPendingInterrupt(.STSPHSRXM, ep_irqs)) {
                    ep.clearInterrupt(.STSPHSRXM);
                }
            }
        }
        fn otgIrqHandler(core: *const USB) void {
            const gotgint = Register{ .addr = core.getReg(.GOTGINT) };
            gotgint.set(gotgint.get());
        }
        fn suspendIrqHandler(core: *const USB) void {
            defer core.clearInterrupt(.UsbSuspend);
            (Field{ .reg = core.getReg(.PCGCCTL), .width = 1, .rw = .ReadWrite, .shift = 0 }).set(1);
        }
        fn rxFifioIrqHandler(core: *const USB) void {
            core.maskInterrupt(.RxFifoNonEmpty);
            defer core.unmaskInterrupt(.RxFifoNonEmpty);

            // pop once
            const grxstsp = (Register{ .addr = core.getReg(.GRXSTSP) }).get();
            // use multiple
            const ep_idx: usize = (Field{ .reg = @intFromPtr(&grxstsp), .width = 4, .rw = .ReadOnly, .shift = 0 }).get();
            const data_size: usize = (Field{ .reg = @intFromPtr(&grxstsp), .width = 11, .rw = .ReadOnly, .shift = 4 }).get();
            const data_type: PacketType = @enumFromInt((Field{ .reg = @intFromPtr(&grxstsp), .width = 4, .rw = .ReadOnly, .shift = 17 }).get());

            if (ep_idx >= core.out_ep.len) {
                return;
            }

            switch (data_type) {
                .DataReceived, .SetupDataReceived => {
                    core.out_ep[ep_idx].read(data_size);
                },
                else => {},
            }
        }
        fn usbResetIrqHandler(core: *const USB) void {
            defer core.clearInterrupt(.UsbReset);
            for (core.in_ep[0..]) |*ep| {
                ep.init();
            }
            core.in_ep[0].unmaskInterrupts();
            core.out_ep[0].unmaskInterrupts();
            core.enableEndpointInterrupts(&.{ .TOC, .XFRCM }, &.{ .STUPM, .XFRCM });
            core.setDeviceAddress(0);
            core.out_ep[0].prepareSetupRx();
        }
        fn enumerationIrqDoneHandler(core: *const USB, ctx: *const IrqHandlerContext) void {
            defer core.clearInterrupt(.EnumerationDone);
            const ep0size = @intFromEnum(ctx.device.ep0Size);
            core.activateSetup(ep0size);
            core.in_ep[0].activate(ep0size, .Control);
            core.out_ep[0].activate(ep0size, .Control);
            core.clearInterrupt(.EnumerationDone);
            core.unmaskInterrupt(.RxFifoNonEmpty);
            core.unmaskInterrupt(.InEndpoint);
            core.unmaskInterrupt(.OutEndpoint);
        }
    };

    fn coreInit(self: *const USB) void {
        self.clearInterrupt(.RxFifoNonEmpty);
        self.setCapability(.HostNegotiationProtocol, .Incapable);
        self.setCapability(.SessionRequestProtocol, .Incapable);
        self.setSpeed(.FullSpeed);
        self.setTimeoutCalibration(4);
        self.unmaskInterrupt(.OTG);
        self.unmaskInterrupt(.ModeMismatch);
    }
    fn dataInit(self: *const USB) void {
        for (self.in_ep[0..], 0..) |*in, i| {
            in.idx = i;
            in.usb = self;
        }

        for (self.out_ep[0..], 0..) |*out, i| {
            out.idx = i;
            out.usb = self;
            out.data = out.buffer[0..0];
        }
    }

    fn deviceInit(self: *const USB) void {
        self.setMode(.Device);
        self.unmaskInterrupt(.UsbReset);
        self.unmaskInterrupt(.EnumerationDone);
        self.unmaskInterrupt(.SOF);
        self.disableVBusSensing();
    }

    pub fn init(self: *const USB, mode: Mode, fifo_config: FifoConfig) void {
        self.dataInit();
        self.coreInit();
        switch (mode) {
            .Device => self.deviceInit(),
        }
        self.disconnect();
        self.configureFifo(fifo_config);
    }
};

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
            .MostBit => field.set(@ctz(value) + (@as(BusType, 1) << (self.width - 1)) - 1 - @as(BusType, 1) * @min(value / 64, 1)),
        }
    }
    fn get(self: *const PRESCALER) BusType {
        const value = (Field{ .reg = rcc.getReg(self.cfg), .width = self.width, .shift = self.shift }).get();

        return switch (self.value_type) {
            .Direct => value - 2,
            .MostBit => @as(BusType, 1) << @truncate(value - @min(value, 7 - @as(BusType, 1) * @min(value / 12, 1))),
        };
    }
};

const PLL = struct {
    cfg: RCC.Reg,
    en_bit: FieldShiftType,
    rdy_bit: FieldShiftType,
    src: @TypeOf(enum {}),

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

    pub fn configure(self: *const PLL, comptime src: self.src, comptime m: u6, comptime n: u9, comptime p: ?u4, comptime q: ?u4, comptime r: ?u3) void {
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
        const rdy = Field{ .reg = rcc.getReg(.CR), .width = 1, .shift = self.rdy_bit };

        if (en.isAsserted())
            en.set(0);

        while (rdy.isAsserted()) {}

        const cfg = rcc.getReg(self.cfg);
        (Field{ .reg = cfg, .shift = 22, .width = 1 }).set(@intFromEnum(src));
        (Field{ .reg = cfg, .shift = 0, .width = 6 }).set(m);
        (Field{ .reg = cfg, .shift = 6, .width = 9 }).set(n);
        if (p != null)
            (Field{ .reg = cfg, .shift = 16, .width = 4 }).set(p.? / 2 - 1);
        if (q != null)
            (Field{ .reg = cfg, .shift = 24, .width = 4 }).set(q.?);
        if (r != null)
            (Field{ .reg = cfg, .shift = 28, .width = 3 }).set(r.?);

        en.set(1);

        while (rdy.isCleared()) {}
    }
};

const FLASH = struct {
    port: BusType,
    usingnamespace CommonInterface(@This());
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
    pub fn setLatency(self: *const FLASH, latency: u4) void {
        (Field{ .reg = self.getReg(.ACR), .shift = 0, .width = 4 }).set(latency);
    }
};

const DWT = struct {
    port: BusType,
    usingnamespace CommonInterface(@This());
    fn enableCycleCounter(self: *const DWT) !void {
        try scs.write(SCS.Trace.Enabled);
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

const NVIC = struct {
    port: BusType,
    prio_bits: u4 = 4,
    usingnamespace CommonInterface(@This());
    const Reg = enum(BusType) {
        ISER = 0x0, // Enable bits space
        IP = 0x300, // Priotiry u8 address space
    };
    const GroupingPriority = enum(u3) {
        Group0 = 7,
        Group1 = 6,
        Group2 = 5,
        Group3 = 4,
        Group4 = 3,
    };
    const Interrupt = enum(u8) { // TODO: add support for cortex exceptions
        OTG_FS = 67,
    };

    pub fn enable(self: *const NVIC, irq: Interrupt) void {
        const field = Field{ .reg = self.getReg(.ISER) + (@intFromEnum(irq) >> 5) * 4, .rw = .WriteOnly, .width = 1, .shift = @truncate(@intFromEnum(irq) & 0x1F) };
        field.set(1);
    }

    pub fn setPriority(self: *const NVIC, irq: Interrupt, preempt_prio: u4, sub_prio: u4) !void {
        @as(*volatile u8, @ptrFromInt(self.getReg(.IP) + @intFromEnum(irq))).* = try self.encodePriority(preempt_prio, sub_prio);
    }

    fn encodePriority(self: *const NVIC, preempt_prio: u4, sub_prio: u4) !u8 {
        const grouping_prio = try scs.read(SCS.InterruptGroupingPriority);
        const preempt_prio_bits = blk: {
            if (0x7 - grouping_prio > self.prio_bits)
                break :blk self.prio_bits;
            break :blk 0x7 - self.prio_bits;
        };

        const sub_prio_bits = blk: {
            if (grouping_prio + self.prio_bits < 0x7)
                break :blk 0;
            break :blk grouping_prio + self.prio_bits - 0x7;
        };

        return @truncate(((preempt_prio & ((@as(u32, 1) << preempt_prio_bits) - 1)) << sub_prio_bits) | (sub_prio & ((@as(u32, 1) << sub_prio_bits) - 1)));
    }

    pub fn setGroupingPriority(self: *const NVIC, prio: GroupingPriority) !void {
        _ = self; // autofix
        try scs.write(SCS.InterruptGroupingPriority{ .value = @intFromEnum(prio) });
    }
};

const MUXER = struct {
    src: Flag,
    rdy: ?Flag = null,
    values: @TypeOf(enum {}),
    const Flag = struct {
        reg: RCC.Reg,
        shift: FieldShiftType,
        width: FieldWidthType = 1,
    };
    pub fn set(self: *const MUXER, value: self.values) void {
        (Field{ .reg = rcc.getReg(self.src.reg), .shift = self.src.shift, .width = self.src.width }).set(@intFromEnum(value));
        if (self.rdy != null) {
            const rdy = Field{ .reg = rcc.getReg(self.rdy.?.reg), .shift = self.rdy.?.shift, .width = self.rdy.?.width };
            while (rdy.get() != @intFromEnum(value)) {}
        }
    }
    fn get(self: *const MUXER) self.values {
        const field: Field = Field{ .reg = rcc.getReg(self.src.reg), .shift = self.src.shift, .width = self.src.width };
        return @enumFromInt(field.get());
    }
};

const POWER = struct {
    port: BusType,
    usingnamespace CommonInterface(@This());

    pub fn configure(self: *const POWER, comptime cpu_hz: usize) void {
        const mode = switch (cpu_hz) {
            0...64_000_000 => 1,
            64_000_001...84_000_000 => 2,
            else => 3,
        };
        (Field{ .reg = self.getReg(.CR), .width = 2, .rw = .ReadWrite, .shift = 14 }).set(mode); // VOS
    }

    const Reg = enum(BusType) {
        CR = 0x0,
        CSR = 0x4,
    };
};

const RCC = struct {
    port: BusType,
    usingnamespace CommonInterface(@This());

    const ClockSource = enum { HSI, HSE };
    const PeripherySwitch = struct {
        en_reg: Reg,
        shift: FieldShiftType,
    };

    const HSEMode = enum { Crystal };

    pub fn enableHSE(self: *const RCC, mode: HSEMode, comptime fq: u32) void {
        hse_fq_hz = fq;

        switch (mode) {
            .Crystal => {
                const cr = self.getReg(.CR);
                const hseon: Field = .{ .reg = cr, .shift = 16, .width = 1 };
                const hserdy: Field = .{ .reg = cr, .rw = .ReadOnly, .shift = 17, .width = 1 };
                const hsebyp: Field = .{ .reg = cr, .shift = 18, .width = 1 };
                hseon.set(0);
                while (hserdy.isAsserted()) {}
                hsebyp.set(0);
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

pub const PERIPHERY = struct {
    pub usingnamespace platform.SwitchInterface(@This(), 0x40020000 + 0x3800);
    pub const SPI1: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.APB2ENR, 12);
    } = .{};
    pub const GPIOA: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.AHB1ENR, 0);
    } = .{};
    pub const GPIOB: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.AHB1ENR, 1);
    } = .{};
    pub const GPIOC: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.AHB1ENR, 2);
    } = .{};
    pub const GPIOD: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.AHB1ENR, 3);
    } = .{};
    pub const GPIOE: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.AHB1ENR, 4);
    } = .{};
    pub const GPIOH: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.AHB1ENR, 7);
    } = .{};
    pub const POWER: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.APB1ENR, 28);
    } = .{};
    pub const OTGUSB: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.AHB2ENR, 7);
    } = .{};
    pub const SYSCONFIG: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.APB2ENR, 14);
    } = .{};
    pub const TIM1: struct {
        pub usingnamespace Field.Interface(@This(), RCC.Reg.APB2ENR, 0);
    } = .{};
};
