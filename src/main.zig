const bus = @import("stm32f411xe.zig");

pub fn main() void {
    bus.rcc.enableHSE(.Crystal, 8_000_000);
    bus.mux.main_pll.set(bus.mux.main_pll.values.HSE);
    bus.pll.main.configure(4, 192, 4, 8, null);
    bus.mux.sys_clock.set(bus.mux.sys_clock.values.PLL);
    bus.presc.ahb.set(1);
    bus.presc.apb1.set(1);
    bus.presc.apb2.set(2);
    bus.enableCycleCounter();
    const sys_clock_hz = bus.getSysClockHz();
    bus.udelay(100_000, sys_clock_hz);

    while (true) {}
}
