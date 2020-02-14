ENTRY(_start)
_estack = 0x20005000;    /* end of RAM */

MEMORY{
    RAM(xrw)   : ORIGIN = 0x20000000, LENGTH = 20K
    FLASH(rx)  : ORIGIN = 0x08000000, LENGTH = 128K
}

SECTIONS{
  .text : {
    *(.vector_table*)
    . = ALIGN(4);
    *(.text)
    *(.text*)
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .rodata : {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  _sidata = LOADADDR(.data);
  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  . = ALIGN(4);
  .bss : {
    _sbss = .;
    *(.bss)
    *(.bss*)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /DISCARD/ : {
    *(*)
  }
}
