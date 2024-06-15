ENTRY(reset_handler)

MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
}

EXTERN (isr_vector)
EXTERN (stack_bottom_addr);
stack_size = 0x1000;

SECTIONS
{
  .isr_vector ALIGN(4): {
    KEEP(*(.isr_vector))
  } >RAM

  .reset_handler ALIGN(4): {
      . = 0x200001B0;
  } > RAM

  .text : ALIGN(4) {
      *(.text)
  } >RAM

  .rodata : ALIGN(4) {
      *(.rodata)
      *(.rodata.*)
  } >RAM

  .bss : ALIGN(4) {
      *(COMMON)
      *(.bss)
  } >RAM

  .stack  : ALIGN(4) {
       . = . + stack_size;
   } >RAM

  /DISCARD/ : { *(.data) }
}

PROVIDE(stack_bottom_addr = ORIGIN(RAM) + LENGTH(RAM));
INCLUDE provide.ld
