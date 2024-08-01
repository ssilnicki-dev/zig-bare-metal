pub const UsbProtoError = error{
    Unsupported,
    OOM,
};

pub fn getSetupRequest(input: []align(4) u8) UsbProtoError!SetupRequest {
    if (input.len < 8)
        return UsbProtoError.Unsupported;

    var setup_request: SetupRequest = undefined;
    const raw_data_layout: *const SetupRequest.raw_data_layout = @ptrCast(input.ptr);
    setup_request.length = raw_data_layout.wlength;
    setup_request.value = raw_data_layout.wvalue;
    setup_request.index = raw_data_layout.windex;

    switch (try parse(SetupRequest.StandardRequestType, raw_data_layout.bmrequest & 0x1F)) {
        .Device => setup_request.request = .{ .device = try parse(SetupRequest.DeviceRequest, raw_data_layout.brequest) },
        .Interface => setup_request.request = .{ .interface = try parse(SetupRequest.InterfaceRequest, raw_data_layout.brequest) },
        .Endpoint => setup_request.request = .{ .endpoint = try parse(SetupRequest.EndpointRequest, raw_data_layout.brequest) },
    }

    return setup_request;
}

fn parse(T: type, rq_hdr: u8) UsbProtoError!T {
    inline for (@typeInfo(T).Enum.fields) |f| {
        if (rq_hdr == f.value)
            return @as(T, @enumFromInt(f.value));
    }
    return UsbProtoError.Unsupported;
}

pub const SetupRequest = struct {
    request: union(enum) {
        device: DeviceRequest,
        interface: InterfaceRequest,
        endpoint: EndpointRequest,
    },
    value: u16,
    length: u16,
    index: u16,

    const raw_data_layout = extern struct {
        bmrequest: u8,
        brequest: u8,
        wvalue: u16,
        windex: u16,
        wlength: u16,
    };

    const StandardRequestType = enum(u7) {
        Device = 0x0,
        Interface = 0x1,
        Endpoint = 0x2,
    };

    const DeviceRequest = enum(u8) {
        GetStatus = 0x0,
        ClearFeature = 0x1,
        SetFeature = 0x3,
        SetAddress = 0x5,
        GetDescriptor = 0x6,
        SetDescriptor = 0x7,
        GetConfiguration = 0x8,
        SetConfiguration = 0x9,
    };

    const InterfaceRequest = enum(u8) {
        GetStatus = 0x0,
        ClearFeature = 0x1,
        SetFeature = 0x3,
        GetInterface = 0x0A,
        SetInterface = 0x11,
    };

    const EndpointRequest = enum(u8) {
        GetStatus = 0x0,
        ClearFeature = 0x1,
        SetFeature = 0x3,
        SyncFrame = 0x12,
    };

    pub const DescriptorType = enum(u8) {
        Device = 0x1,
        Configuration = 0x2,
        String = 0x3,
        Interface = 0x4,
        Endpoint = 0x5,
        DeviceQualifier = 0x6,
        OtherSpeedConfiguration = 0x7,
        IAD = 0xB,
        BOS = 0xF,
    };

    fn getDescriptor(self: SetupRequest) UsbProtoError!DescriptorType {
        return try parse(DescriptorType, @truncate(self.value >> 8));
    }
};

pub const UsbDevice = struct {
    lpm: bool,
    class: Class,
    subClass: u8,
    protocol: u8,
    ep0Size: EP0Size,
    vendorId: u16,
    productId: u16,
    vendorNameStringId: u8,
    productNameStringId: u8,
    serialNumberStringId: u8,
    vendorName: []const u8,
    productName: []const u8,
    configurations: []const Configuration,
    languages: []const Language = &.{.EnglishUS},
    serialNumberGetter: SerialNumberGetter,

    pub const SerialNumberGetter = *const fn ([]u8) UsbProtoError![]const u8;

    pub const Language = enum(u16) {
        EnglishUS = 0x409,
    };

    fn getLanguagesDescriptor(self: UsbDevice, buf: []u8) ![]const u8 {
        const size = self.languages.len * 2 + 2;
        if (size > buf.len) return UsbProtoError.OOM;
        if (size > 255) return UsbProtoError.Unsupported;
        buf[0] = @truncate(size);
        buf[1] = @intFromEnum(SetupRequest.DescriptorType.String);
        var sub_buf = buf[2..];
        for (self.languages) |lang| {
            sub_buf[0] = @truncate(@intFromEnum(lang));
            sub_buf[1] = @truncate(@intFromEnum(lang) >> 8);
            sub_buf = sub_buf[2..];
        }
        return buf[0..size];
    }

    fn encodeUTF16StringDescriptor(src: []const u8, buf: []u8) ![]const u8 {
        const size = src.len * 2 + 2;
        if (size > buf.len) return UsbProtoError.OOM;
        if (src.len > 127) return UsbProtoError.Unsupported;
        buf[0] = @truncate(size);
        buf[1] = @intFromEnum(SetupRequest.DescriptorType.String);
        var dst = buf[2..];
        for (src, 0..) |ch, i| {
            dst[i * 2] = ch;
            dst[i * 2 + 1] = 0x0;
        }
        return buf[0..size];
    }

    fn getStringDescriptor(self: UsbDevice, buf: []u8, idx: u8) ![]const u8 {
        if (idx == 0) {
            return try self.getLanguagesDescriptor(buf);
        } else if (idx == self.vendorNameStringId) {
            return try encodeUTF16StringDescriptor(self.vendorName, buf);
        } else if (idx == self.productNameStringId) {
            return try encodeUTF16StringDescriptor(self.productName, buf);
        } else if (idx == self.serialNumberStringId) {
            const serial = try self.serialNumberGetter(buf[2..]);
            const size = serial.len + 2;
            buf[0] = @truncate(size);
            buf[1] = @intFromEnum(SetupRequest.DescriptorType.String);
            return buf[0..size];
        } else {
            if (buf.len < 2) return UsbProtoError.OOM;
            buf[0] = 2;
            buf[1] = @intFromEnum(SetupRequest.DescriptorType.String);
            return buf[0..2];
        }
    }

    pub fn getConfiguration(self: UsbDevice, idx: usize) !Configuration {
        if (idx >= self.configurations.len)
            return UsbProtoError.Unsupported;
        return self.configurations[idx];
    }

    pub const Configuration = struct {
        const Feature = enum(u3) {
            SelfPowered = 6,
            RemoteWakeup = 5,
        };
        features: []Feature = &.{},
        milliAmperes: u9,
        interfaces: []const Interface,
        fn getSize(self: Configuration) usize {
            var size: usize = 9;
            for (self.interfaces) |*desc| {
                size += desc.getSize();
            }
            return size;
        }

        fn getDescriptor(self: Configuration, buf: []u8, cfg_idx: u8) ![]const u8 {
            const size = self.getSize();
            if (size > buf.len) return UsbProtoError.OOM;
            if (self.interfaces.len > 255) return UsbProtoError.Unsupported;
            buf[0] = 9;
            buf[1] = @intFromEnum(SetupRequest.DescriptorType.Configuration);
            buf[2] = @truncate(size);
            buf[3] = @truncate(size >> 8);
            buf[4] = @min(255, self.interfaces.len);
            buf[5] = cfg_idx;
            buf[6] = 0x0;
            buf[7] = 0x80;
            for (self.features) |f| buf[7] |= @as(u8, 1) << @intFromEnum(f);
            buf[8] = @truncate(self.milliAmperes >> 1);
            var sub_buf = buf[9..];
            for (self.interfaces, 0..) |*desc, if_idx| sub_buf = sub_buf[(try desc.getDescriptor(sub_buf, @truncate(if_idx))).len..];
            return buf[0..size];
        }
    };

    pub const Interface = struct {
        settings: []const AltSetting,
        pub const AltSetting = struct {
            classDescriptors: []const ClassSpecificDescriptor = &[_]ClassSpecificDescriptor{},
            endpoints: []const Endpoint,
            class: Class,
            subClass: u8,
            protocol: u8,
            pub const ClassSpecificDescriptor = struct {
                descriptorType: u8,
                data: []const u8,
                fn getSize(self: ClassSpecificDescriptor) usize {
                    return self.data.len + 2;
                }
                fn getDescriptor(self: ClassSpecificDescriptor, buf: []u8) ![]const u8 {
                    const size = self.getSize();
                    if (size > buf.len) return UsbProtoError.OOM;
                    if (size > 255) return UsbProtoError.Unsupported;
                    buf[0] = @truncate(size);
                    buf[1] = self.descriptorType;
                    for (self.data[0..], 2..) |b, i| {
                        buf[i] = b;
                    }
                    return buf[0..buf[0]];
                }
            };

            fn getSize(self: AltSetting) usize {
                var size: usize = 9;
                for (self.classDescriptors) |*desc| {
                    size += desc.getSize();
                }
                size += self.endpoints.len * Endpoint.getSize();
                return size;
            }

            fn getDescriptor(self: AltSetting, buf: []u8, if_idx: u8, alt_idx: u8) ![]const u8 {
                const size = self.getSize();
                if (size > buf.len) return UsbProtoError.OOM;
                buf[0] = 9;
                buf[1] = @intFromEnum(SetupRequest.DescriptorType.Interface);
                buf[2] = if_idx;
                buf[3] = alt_idx;
                buf[4] = @min(255, self.endpoints.len);
                buf[5] = @intFromEnum(self.class);
                buf[6] = self.subClass;
                buf[7] = self.protocol;
                buf[8] = if_idx;
                var sub_buf = buf[9..];
                for (self.classDescriptors) |*desc| sub_buf = sub_buf[(try desc.getDescriptor(sub_buf)).len..];
                for (self.endpoints) |*desc| sub_buf = sub_buf[(try desc.getDescriptor(sub_buf)).len..];
                return buf[0..size];
            }
        };
        fn getSize(self: Interface) usize {
            var size: usize = 0;
            for (self.settings) |*desc| {
                size += desc.getSize();
            }
            return size;
        }

        fn getDescriptor(self: Interface, buf: []u8, if_idx: u8) ![]const u8 {
            const size = self.getSize();
            if (size > buf.len) return UsbProtoError.OOM;
            if (self.settings.len > 255) return UsbProtoError.Unsupported;
            var sub_buf = buf;
            for (self.settings, 0..) |*desc, alt_idx| {
                sub_buf = sub_buf[(try desc.getDescriptor(sub_buf, if_idx, @truncate(alt_idx))).len..];
            }
            return buf[0..size];
        }
    };

    pub const Endpoint = struct {
        pub const Direction = enum(u8) { In = 0x80, Out = 0x0 };
        pub const Type = enum(u2) { Control = 0, Isochronous = 1, Bulk = 2, Interrupt = 3 };
        pub const IsocSyncType = enum(u2) { NoSync = 0, Async = 1, Adaptive = 2, Sync = 3 };
        pub const IsocType = enum(u2) { Data = 0, Feedback = 1, ExplicitFeedback = 2 };
        address: u4,
        direction: Direction,
        type: Type,
        isocType: ?IsocType = null,
        isocSyncType: ?IsocSyncType = null,
        packetSize: u16,
        interval: u8 = 0,
        inline fn getSize() u8 {
            return comptime 7;
        }
        fn getDescriptor(self: Endpoint, buf: []u8) ![]const u8 {
            const size = getSize();
            if (size > buf.len) return UsbProtoError.OOM;
            buf[0] = size; // descriptor length
            buf[1] = @intFromEnum(SetupRequest.DescriptorType.Endpoint);
            buf[2] = @as(u8, self.address) + @intFromEnum(self.direction);
            buf[3] = @intFromEnum(self.type);
            if (self.type == .Isochronous) {
                buf[3] |= @as(u8, @intFromEnum(self.isocType orelse .Data)) << 2;
                buf[3] |= @as(u8, @intFromEnum(self.isocSyncType orelse .NoSync)) << 4;
            }
            buf[4] = @truncate(self.packetSize);
            buf[5] = @truncate(self.packetSize >> 8);
            buf[6] = self.interval;
            return buf[0..buf[0]];
        }
    };

    const EP0Size = enum(u8) {
        EP0Size8 = 8,
        EP0Size16 = 16,
        EP0Size32 = 32,
        EP0Size64 = 64,
    };

    const Class = enum(u8) {
        // https://www.usb.org/defined-class-codes
        Vendor = 0xFF,
    };
    pub const SubClass = enum(u8) {
        Application = 0xFE,
    };

    fn getDeviceDescriptor(self: UsbDevice, buf: []u8) ![]const u8 {
        const size = 0x12;
        if (size > buf.len) return UsbProtoError.OOM;
        buf[0] = size;
        buf[1] = @intFromEnum(SetupRequest.DescriptorType.Device);
        buf[2] = @intFromBool(self.lpm); // bcdUSB
        buf[3] = 0x2; // bcdUSB
        buf[4] = @intFromEnum(self.class);
        buf[5] = self.subClass;
        buf[6] = self.protocol;
        buf[7] = @intFromEnum(self.ep0Size);
        buf[8] = @truncate(self.vendorId);
        buf[9] = @truncate(self.vendorId >> 8);
        buf[10] = @truncate(self.productId);
        buf[11] = @truncate(self.productId >> 8);
        buf[12] = 0; // bcdDevice
        buf[13] = 2; // bcdDevice
        buf[14] = self.vendorNameStringId;
        buf[15] = self.productNameStringId;
        buf[16] = self.serialNumberStringId;
        buf[17] = @truncate(self.configurations.len);
        return buf[0..buf[0]];
    }
    fn getDeviceQualifierDescriptor(self: UsbDevice, buf: []u8) ![]const u8 {
        const size = 0x0A;
        if (size > buf.len) return UsbProtoError.OOM;
        buf[0] = size;
        buf[1] = @intFromEnum(SetupRequest.DescriptorType.DeviceQualifier);
        buf[2] = @intFromBool(self.lpm); // bcdUSB
        buf[3] = 0x2; // bcdUSB
        buf[4] = @intFromEnum(self.class);
        buf[5] = self.subClass;
        buf[6] = self.protocol;
        buf[7] = @intFromEnum(self.ep0Size);
        buf[8] = @truncate(self.configurations.len);
        buf[9] = 0x0;
        return buf[0..buf[0]];
    }

    pub fn getDescriptor(self: UsbDevice, req: SetupRequest, buf: []u8) ![]const u8 {
        switch (try req.getDescriptor()) {
            .String => return try self.getStringDescriptor(buf, @truncate(req.value)),
            .Configuration => {
                if (req.index >= self.configurations.len)
                    return UsbProtoError.Unsupported;
                return (try self.configurations[req.index].getDescriptor(buf, @truncate(req.index)))[0..req.length];
            },
            .Device => return try self.getDeviceDescriptor(buf),
            .DeviceQualifier => return try self.getDeviceQualifierDescriptor(buf),
            else => return UsbProtoError.Unsupported,
        }
    }
};
