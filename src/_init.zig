const main = @import("main.zig");

export fn blocking_handler() void {
    while (true) {}
}
export fn null_handler() void {}

export fn _start() void {
    main.main();
}
export fn reset_handler() void {
    _start();
    unreachable;
}

extern fn stack_bottom_addr() void;
export const isr_vector linksection(".isr_vector") = [_]?*const fn () callconv(.C) void{
    stack_bottom_addr,
    reset_handler,
    nmi_handler,
    hard_fault_handler,
    mem_manage_fault_handler,
    bus_fault_handler,
    usage_fault_handler,
    null,
    null,
    null,
    null,
    svc_handler,
    debug_mon_handler,
    null,
    pend_sv_handler,
    systick_handler,

    wwdg_irq,
    pvd_irq,
    tamp_stamp_irq,
    rtc_wkup_irq,
    flash_irq,
    rcc_irq,
    exti0_irq,
    exti1_irq,
    exti2_irq,
    exti3_irq,
    exti4_irq,
    dma1_stream0_irq,
    dma1_stream1_irq,
    dma1_stream2_irq,
    dma1_stream3_irq,
    dma1_stream4_irq,
    dma1_stream5_irq,
    dma1_stream6_irq,
    adc_irq,
    null,
    null,
    null,
    null,
    exti9_5_irq,
    tim1_brk_tim9_irq,
    tim1_up_tim10_irq,
    tim1_trg_com_tim11_irq,
    tim1_cc_irq,
    tim2_irq,
    tim3_irq,
    tim4_irq,
    i2c1_ev_irq,
    i2c1_er_irq,
    i2c2_ev_irq,
    i2c2_er_irq,
    spi1_irq,
    spi2_irq,
    usart1_irq,
    usart2_irq,
    null,
    exti15_10_irq,
    rtc_alarm_irq,
    otg_fs_wkup_irq,
    null,
    null,
    null,
    null,
    dma1_stream7_irq,
    null,
    sdio_irq,
    tim5_irq,
    spi3_irq,
    null,
    null,
    null,
    null,
    dma2_stream0_irq,
    dma2_stream1_irq,
    dma2_stream2_irq,
    dma2_stream3_irq,
    dma2_stream4_irq,
    null,
    null,
    null,
    null,
    null,
    null,
    otg_fs_irq,
    dma2_stream5_irq,
    dma2_stream6_irq,
    dma2_stream7_irq,
    usart6_irq,
    i2c3_ev_irq,
    i2c3_er_irq,
    null,
    null,
    null,
    null,
    null,
    null,
    null,
    fpu_irq,
    null,
    null,
    spi4_irq,
    spi5_irq,
};

extern fn nmi_handler() void;
extern fn hard_fault_handler() void;
extern fn mem_manage_fault_handler() void;
extern fn bus_fault_handler() void;
extern fn usage_fault_handler() void;
extern fn svc_handler() void;
extern fn debug_mon_handler() void;
extern fn pend_sv_handler() void;
extern fn systick_handler() void;
extern fn wwdg_irq() void;
extern fn pvd_irq() void;
extern fn tamp_stamp_irq() void;
extern fn rtc_wkup_irq() void;
extern fn flash_irq() void;
extern fn rcc_irq() void;
extern fn exti0_irq() void;
extern fn exti1_irq() void;
extern fn exti2_irq() void;
extern fn exti3_irq() void;
extern fn exti4_irq() void;
extern fn dma1_stream0_irq() void;
extern fn dma1_stream1_irq() void;
extern fn dma1_stream2_irq() void;
extern fn dma1_stream3_irq() void;
extern fn dma1_stream4_irq() void;
extern fn dma1_stream5_irq() void;
extern fn dma1_stream6_irq() void;
extern fn adc_irq() void;
extern fn exti9_5_irq() void;
extern fn tim1_brk_tim9_irq() void;
extern fn tim1_up_tim10_irq() void;
extern fn tim1_trg_com_tim11_irq() void;
extern fn tim1_cc_irq() void;
extern fn tim2_irq() void;
extern fn tim3_irq() void;
extern fn tim4_irq() void;
extern fn i2c1_ev_irq() void;
extern fn i2c1_er_irq() void;
extern fn i2c2_ev_irq() void;
extern fn i2c2_er_irq() void;
extern fn spi1_irq() void;
extern fn spi2_irq() void;
extern fn usart1_irq() void;
extern fn usart2_irq() void;
extern fn exti15_10_irq() void;
extern fn rtc_alarm_irq() void;
extern fn otg_fs_wkup_irq() void;
extern fn dma1_stream7_irq() void;
extern fn sdio_irq() void;
extern fn tim5_irq() void;
extern fn spi3_irq() void;
extern fn dma2_stream0_irq() void;
extern fn dma2_stream1_irq() void;
extern fn dma2_stream2_irq() void;
extern fn dma2_stream3_irq() void;
extern fn dma2_stream4_irq() void;
extern fn otg_fs_irq() void;
extern fn dma2_stream5_irq() void;
extern fn dma2_stream6_irq() void;
extern fn dma2_stream7_irq() void;
extern fn usart6_irq() void;
extern fn i2c3_ev_irq() void;
extern fn i2c3_er_irq() void;
extern fn fpu_irq() void;
extern fn spi4_irq() void;
extern fn spi5_irq() void;
