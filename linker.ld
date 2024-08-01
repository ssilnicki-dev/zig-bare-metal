ENTRY(reset_handler)

MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH (rx)     : ORIGIN = 0x8000000, LENGTH = 512K
}

EXTERN (isr_vector)
EXTERN (stack_bottom_addr);
stack_size = 0x1000;

SECTIONS
{
  .isr_vector ALIGN(4): {
    KEEP(*(.isr_vector))
  } >FLASH

  .reset_handler ALIGN(4): {
      . = 0x80001B0;
  } > FLASH

  .text : ALIGN(4) {
      KEEP(*(.text.__aeabi_memset))
      KEEP(*(.text.__aeabi_memcpy))
      KEEP(*(.text.memset))
      *(.text)
  } >FLASH

  .rodata : ALIGN(4) {
      *(.rodata)
      *(.rodata.*)
  } >FLASH

  .bss (NOLOAD) : ALIGN(4) {
      *(COMMON)
      *(.bss)
  } >RAM

  .stack (NOLOAD) : ALIGN(4) {
       . = . + stack_size;
  } >RAM

  /DISCARD/ : { *(.data) }
}

PROVIDE(stack_bottom_addr = ORIGIN(RAM) + LENGTH(RAM));
INCLUDE provide.ld
