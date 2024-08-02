const platform = @import("platform32.zig");
const BusType = platform.BusType;
const FieldShiftType = platform.FieldShiftType;
const FieldWidthType = platform.FieldWidthType;
const Field = platform.Field;

pub const GPIO = struct {
    port: BusType,
    const Reg = enum(BusType) {
        MODER = 0x0,
        OTYPER = 0x4,
        OSPEEDR = 0x8,
        PUPDR = 0xC,
        IDR = 0x10,
        BRSR = 0x18,
        AFR = 0x20,
    };

    const MODE = union(enum) {
        gpio: MODE.GPIO,
        alt_fn: u4,
        const GPIO = enum(u2) { Input = 0, Output = 1, AltFunc = 2, Analog = 3 };
    };
    const OTYPE = enum(u1) { PushPull = 0, OpenDrain = 1 };
    const OSPEED = enum(u2) { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
    const PUPD = enum(u2) { Disabled = 0, PullUp = 1, PullDown = 2, Reserved = 3 };

    pub fn pin(self: *const GPIO, nr: u4) Pin {
        return .{ .gpio = self, .pin = nr };
    }

    pub const Pin = PIN;

    const PIN = struct {
        gpio: *const GPIO,
        pin: u4,

        pub fn configure(self: *const PIN, mode: MODE, otype: OTYPE, ospeed: OSPEED, pupd: PUPD) void {
            switch (mode) {
                .gpio => |gpio| {
                    self.getAFR().set(0);
                    self.getMODER().set(@intFromEnum(gpio));
                },
                .alt_fn => |af| {
                    self.getMODER().set(@intFromEnum(MODE.GPIO.AltFunc));
                    self.getAFR().set(af);
                },
            }

            self.getOTYPER().set(@intFromEnum(otype));
            self.getOSPEEDR().set(@intFromEnum(ospeed));
            self.getPUPDR().set(@intFromEnum(pupd));
        }
        pub fn isAsserted(self: PIN) bool {
            return !(self.getIDR().get() == 0);
        }
        pub fn assert(self: PIN) void {
            self.getBSR().set(1);
        }
        pub fn reset(self: PIN) void {
            self.getBRR().set(1);
        }
        fn getMODER(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.MODER), .rw = .ReadWrite, .shift = self.pin * @as(FieldShiftType, 2), .width = 2 };
        }
        fn getOTYPER(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.OTYPER), .rw = .ReadWrite, .shift = self.pin, .width = 1 };
        }
        fn getOSPEEDR(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.OSPEEDR), .rw = .ReadWrite, .shift = self.pin * @as(FieldShiftType, 2), .width = 2 };
        }
        fn getPUPDR(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.PUPDR), .rw = .ReadWrite, .shift = self.pin * @as(FieldShiftType, 2), .width = 2 };
        }
        fn getAFR(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.AFR) + (self.pin / 8) * 4, .rw = .ReadWrite, .shift = (self.pin % 8) * @as(FieldShiftType, 4), .width = 4 };
        }
        fn getBSR(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.BRSR), .rw = .WriteOnly, .shift = self.pin, .width = 1 };
        }
        fn getBRR(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.BRSR), .rw = .WriteOnly, .shift = @as(FieldShiftType, 0x10) + self.pin, .width = 1 };
        }
        fn getIDR(self: PIN) Field {
            return .{ .reg = self.gpio.port + @intFromEnum(GPIO.Reg.IDR), .rw = .ReadOnly, .shift = self.pin, .width = 1 };
        }
    };
};
