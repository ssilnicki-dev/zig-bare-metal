const hal = @import("stm32f411xe.zig");
const usb = hal.usb;
const usbp = @import("usbp.zig");

pub fn main() void {
    hal.power.configure(96_000_000);
    hal.rcc.enableHSE(.Crystal, 8_000_000);
    hal.pll.main.configure(hal.pll.main.src.HSE, 4, 192, 4, 8, null);
    hal.flash.setLatency(3);
    hal.mux.sys_clock.set(hal.mux.sys_clock.values.PLL);
    hal.presc.ahb.set(1);
    hal.presc.apb1.set(2);
    hal.presc.apb2.set(1);
    hal.enableCycleCounter();
    hal.nvic.setGroupingPriority(.Group4);

    hal.nvic.setPriority(.OTG_FS, 2, 0);
    usb.init(.Device, .{ .rxFifoSize = 0x80, .txFifoSize = &[_]u16{0x80} });
    usb.connect();
}

var usb_device_irq_context: hal.USB.DeviceRoutines.IrqHandlerContext = .{
    .device = &usb_device,
    .setConfiguration = &setConfiguration,
};

fn setConfiguration(cfg_idx: u8, ctx: *hal.USB.DeviceRoutines.IrqHandlerContext) usbp.UsbProtoError!void {
    if (ctx.currentConfiguration != null) {}
    const configuration = try ctx.device.getConfiguration(cfg_idx);
    ctx.currentConfiguration = &configuration;
}

const usb_device = usbp.UsbDevice{
    .lpm = false,
    .ep0Size = .EP0Size64,
    .class = .Vendor,
    .subClass = @intFromEnum(usbp.UsbDevice.SubClass.Application),
    .protocol = 0x0,
    .vendorId = 0x0,
    .productId = 0x0,
    .vendorNameStringId = 0x1,
    .productNameStringId = 0x2,
    .serialNumberStringId = 0x3,
    .vendorName = "Custom Devices",
    .productName = "Specific USB device",
    .serialNumberGetter = hal.USB.DeviceRoutines.serialNumberGetter,
    .configurations = &[_]usbp.UsbDevice.Configuration{.{
        .milliAmperes = 100,
        .interfaces = &[_]usbp.UsbDevice.Interface{.{
            .settings = &[_]usbp.UsbDevice.Interface.AltSetting{
                .{
                    .class = .Vendor,
                    .subClass = @intFromEnum(usbp.UsbDevice.SubClass.Application),
                    .protocol = 0x0,
                    .endpoints = &[_]usbp.UsbDevice.Endpoint{
                        .{ .address = 1, .direction = .In, .type = .Bulk, .packetSize = 64, .interval = 0 },
                        .{ .address = 1, .direction = .Out, .type = .Bulk, .packetSize = 64, .interval = 0 },
                    },
                },
            },
        }},
    }},
};

export fn otg_fs_irq() void {
    usb.device.otgFsIrqHandler(&usb_device_irq_context);
}
