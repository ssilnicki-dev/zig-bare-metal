const bus = @import("stm32f411xe.zig");

pub export fn flash_init() void {
    bus.flash.enableInstructionCache();
    bus.flash.enableDataCache();
    bus.flash.enablePrefetchBuffer();
}
