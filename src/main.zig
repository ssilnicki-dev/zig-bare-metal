const bus = @import("stm32f411xe.zig");

pub fn main() void {
    bus.rcc.enableHSE(.Crystal, 8_000_000);
    while (true) {}
}
