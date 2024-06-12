const hal = @import("stm32f411xe.zig");

pub fn main() void {
    hal.rcc.enableHSE(.Crystal, 8_000_000);
    hal.mux.main_pll.set(hal.mux.main_pll.values.HSE);
    hal.pll.main.configure(4, 192, 4, 8, null);
    hal.mux.sys_clock.set(hal.mux.sys_clock.values.PLL);
    hal.presc.ahb.set(1);
    hal.presc.apb1.set(1);
    hal.presc.apb2.set(2);
    hal.enableCycleCounter();
    const sys_clock_hz = hal.getSysClockHz();
    hal.udelay(100_000, sys_clock_hz);

    while (true) {}
}
