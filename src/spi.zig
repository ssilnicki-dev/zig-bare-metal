const platform = @import("platform32.zig");
const BusType = platform.BusType;
const setting = @import("setting.zig");

pub const SPI = struct {
    port: BusType,

    pub const udelay_type = *const fn (us: comptime_int) callconv(.Inline) void;

    pub usingnamespace setting.ControllerInterface(@This());
    pub usingnamespace platform.PlatformPeripheryInterface(@This());

    pub const Status = SR;
    pub const Data8 = DR.Byte;
    pub const Data16 = DR.HalfWord;
    pub const Control = struct {
        pub const ClockPhase = CR1.ClockPhase;
        pub const ClockPolarity = CR1.ClockPolarity;
        pub const Mode = CR1.Mode;
        pub const ClockDivider = CR1.ClockDivider;
        pub const Periphery = CR1.Periphery;
        pub const FrameFormat = CR1.FrameFormat;
        pub const NSSLevel = CR1.NSSLevel;
        pub const SoftwareSlaveManagement = CR1.SoftwareSlaveManagement;
        pub const RxMode = CR1.RxMode;
        pub const DataFrameFormat = CR1.DataFrameFormat;
        pub const CRCNext = CR1.CRCNext;
        pub const CRCCalc = CR1.CRCCalc;
        pub const LineMode = CR1.LineMode;
        pub const HwFrameFormat = CR2.HwFrameFormat;
        pub const NSSMode = CR2.NSSMode;
    };

    const default_reg_type = u32;
};

pub const Error = error{
    Timeout,
};

const SR = struct {
    const reg = 0x8;
    pub const Rx = enum(u1) {
        NoData = 0,
        DataReady = 1,
        pub usingnamespace setting.ReadInterface(@This(), reg, SPI.default_reg_type, 0, 0);
    };
    pub const Tx = enum(u1) {
        DataPending = 0,
        Empty = 1,
        pub usingnamespace setting.ReadInterface(@This(), reg, SPI.default_reg_type, 0, 1);
    };
    pub const Busy = enum(u1) {
        Set = 1,
        Cleared = 0,
        pub usingnamespace setting.ReadInterface(@This(), reg, SPI.default_reg_type, 0, 7);
    };
};

const DR = struct {
    const reg = 0xC;
    const Byte = struct {
        value: u8 = undefined,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, u8, 0, 0);
    };
    const HalfWord = struct {
        volue: u16 = undefined,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, u16, 0, 0);
    };
};

const CR1 = struct {
    const reg = 0x0;
    const ClockPhase = enum(u1) {
        FirstEdge = 0,
        SecondEdge = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 0);
    };
    const ClockPolarity = enum(u1) {
        Low = 0,
        High = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 1);
    };
    const Mode = enum(u1) {
        Slave = 0,
        Master = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 2);
    };
    const ClockDivider = enum(u3) {
        Div2 = 0,
        Div4 = 1,
        Div8 = 2,
        Div16 = 3,
        Div32 = 4,
        Div64 = 5,
        Div128 = 6,
        Div256 = 7,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 3);
    };
    const Periphery = enum(u1) {
        Enabled = 1,
        Disabled = 0,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 6);
    };
    const FrameFormat = enum(u1) {
        MsbFirst = 0,
        LsbFirst = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 7);
    };
    const NSSLevel = enum(u1) {
        Low = 0,
        High = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 8);
    };
    const SoftwareSlaveManagement = enum(u1) {
        Enabled = 1,
        Disabled = 0,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 9);
    };
    const RxMode = enum(u1) {
        FullDuplex = 0,
        RxOnly = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 10);
    };
    const DataFrameFormat = enum(u1) {
        Byte = 0,
        HalfWord = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 11);
    };
    const CRCNext = enum(u1) {
        Disabled = 0,
        Enabled = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 12);
    };
    const CRCCalc = enum(u1) {
        Disabled = 0,
        Enabled = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 13);
    };
    const LineMode = enum(u2) {
        TwoLinesUnidir = 0,
        OneLineRxOnly = 2,
        OneLineTxOnly = 3,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 14);
    };
};

const CR2 = struct {
    const reg = 0x4;
    const HwFrameFormat = enum(u1) {
        Motorola = 0,
        TI = 1,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 4);
    };
    pub const NSSMode = enum(u1) {
        Master = 1,
        Slave = 0,
        pub usingnamespace setting.ReadWriteInterface(@This(), reg, SPI.default_reg_type, 0, 2);
    };
};

pub fn MasterInterface(comptime T: type, timeout_us: comptime_int) type {
    return struct {
        inline fn selectSpiPrescaler(clock_hz: usize, target_hz: comptime_int) SPI.Control.ClockDivider {
            return switch (clock_hz / target_hz) {
                0...2 => .Div2,
                3...4 => .Div4,
                5...8 => .Div8,
                9...16 => .Div16,
                17...32 => .Div32,
                33...64 => .Div64,
                65...128 => .Div128,
                else => .Div256,
            };
        }

        pub fn setSpiClock(self: T, input_clock_hz: usize, spi_clock_hz: comptime_int) !void {
            try self.spi.write(selectSpiPrescaler(input_clock_hz, spi_clock_hz));
        }

        inline fn waitFor(spi: *const SPI, comptime direction: enum { ReadReady, WriteReady, Idle }, udelay: SPI.udelay_type) !void {
            const status_field = switch (direction) {
                .ReadReady => SPI.Status.Rx,
                .WriteReady => SPI.Status.Tx,
                .Idle => SPI.Status.Busy,
            };
            const expected_status = switch (direction) {
                .ReadReady => SPI.Status.Rx.DataReady,
                .WriteReady => SPI.Status.Tx.Empty,
                .Idle => SPI.Status.Busy.Cleared,
            };

            var waited_us: usize = 0;
            while (try spi.read(status_field) != expected_status and waited_us < timeout_us) {
                udelay(100);
                waited_us += 100;
            }
            if (waited_us >= timeout_us)
                return Error.Timeout;
        }

        pub fn readBytesFromSlave(self: T, data: []u8) !void {
            if (try self.spi.read(SPI.Control.Periphery) != .Enabled)
                try self.spi.write(SPI.Control.Periphery.Enabled);
            for (data) |*byte| {
                try waitFor(self.spi, .WriteReady, self.udelay);
                try self.spi.write(SPI.Data8{ .value = 0 });
                try waitFor(self.spi, .ReadReady, self.udelay);
                byte.* = @truncate(try self.spi.read(SPI.Data8));
            }
            try waitFor(self.spi, .Idle, self.udelay);
        }

        pub fn writeBytesToSlave(self: T, data: []const u8) !void {
            if (try self.spi.read(SPI.Control.Periphery) != .Enabled)
                try self.spi.write(SPI.Control.Periphery.Enabled);
            for (data) |*byte| {
                try waitFor(self.spi, .WriteReady, self.udelay);
                try self.spi.write(SPI.Data8{ .value = byte.* });
            }
            try waitFor(self.spi, .Idle, self.udelay);
        }
        pub fn readHalfWordsFromSlave(self: T, data: anytype) !void {
            if (try self.spi.read(SPI.Control.Periphery) != .Enabled)
                try self.spi.write(SPI.Control.Periphery.Enabled);
            for (data) |*half_word| {
                try waitFor(self.spi, .WriteReady, T.spi_timeout_us, self.udelay);
                try self.spi.write(SPI.Data16{ .value = 0 });
                try waitFor(self.spi, .ReadReady, T.spi_timeout_us, self.udelay);
                half_word.* = @truncate(try self.spi.read(SPI.Data16));
            }
            try waitFor(self.spi, .Idle, T.spi_timeout_us, self.udelay);
        }

        pub fn writeHalfWordsToSlave(self: T, data: anytype) !void {
            if (try self.spi.read(SPI.Control.Periphery) != .Enabled)
                try self.spi.write(SPI.Control.Periphery.Enabled);
            for (data) |*half_word| {
                try waitFor(self.spi, .WriteReady, T.spi_timeout_us, self.udelay);
                try self.spi.write(SPI.Data16{ .value = half_word.* });
            }
            try waitFor(self.spi, .Idle, T.spi_timeout_us, self.udelay);
        }
    };
}
