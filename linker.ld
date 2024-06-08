ENTRY(reset_handler)

MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH (rx)     : ORIGIN = 0x8000000, LENGTH = 512K
}

EXTERN (isr_vector)
EXTERN (stack_bottom_addr);

SECTIONS
{
  .isr_vector ALIGN(4): {
    KEEP(*(.isr_vector))
  } >FLASH

  .text : ALIGN(4) {
      *(.text)
  } >FLASH

  .rodata : ALIGN(4) {
      *(.rodata)
  } >FLASH

  .bss : ALIGN(4) {
      *(COMMON)
      *(.bss)
  } >RAM
  /DISCARD/ : { *(.data) }
}

PROVIDE(stack_bottom_addr = ORIGIN(RAM) + LENGTH(RAM));
stack_size = 0x1000;
INCLUDE provide.ld
