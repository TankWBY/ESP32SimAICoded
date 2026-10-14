/* RAM-only layout: the mask ROM loads every segment straight into SRAM, so
 * this image can sit at flash offset 0x0 like a 2nd stage bootloader does. */
ENTRY(_start)
MEMORY {
    iram (rwx) : ORIGIN = 0x40390000, LENGTH = 64K   /* SRAM1 via the instruction bus */
    dram (rw)  : ORIGIN = 0x3FCA0000, LENGTH = 64K   /* SRAM1 via the data bus, no overlap with iram */
}
SECTIONS {
    .text : { *(.text.entry) *(.text .text.*) } > iram
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } > dram
    .data : { *(.data .data.* .sdata .sdata.*) } > dram
    .bss (NOLOAD) : { _bss_start = .; *(.bss .bss.* .sbss .sbss.* COMMON) . = ALIGN(4); _bss_end = .; } > dram
    __global_pointer$ = ADDR(.rodata) + 0x800;
    _stack_top = 0x3FCDE000;
    /DISCARD/ : { *(.comment) *(.riscv.attributes) }
}
