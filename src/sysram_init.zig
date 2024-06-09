const bus = @import("stm32f411xe.zig");

pub export fn sysram_init() void {
    bus.scb.setSramVtor();
}
