/* Application layout: code and constants execute in place from flash through
 * the cache MMU (IROM/DROM windows), variables live in SRAM. The IROM and
 * DROM windows share one MMU table on the C3, so they use different 64 KiB
 * page indices. Flash it at 0x10000 with a partition table at 0x8000. */
ENTRY(_start)
MEMORY {
    irom (rx) : ORIGIN = 0x42000020, LENGTH = 64K - 0x20
    drom (r)  : ORIGIN = 0x3C010020, LENGTH = 64K - 0x20
    dram (rw) : ORIGIN = 0x3FCA0000, LENGTH = 64K
}
SECTIONS {
    .text : { *(.text.entry) *(.text .text.*) } > irom
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } > drom
    .data : { *(.data .data.* .sdata .sdata.*) } > dram
    .bss (NOLOAD) : { _bss_start = .; *(.bss .bss.* .sbss .sbss.* COMMON) . = ALIGN(4); _bss_end = .; } > dram
    __global_pointer$ = ADDR(.data) + 0x800;
    _stack_top = 0x3FCDE000;
    /DISCARD/ : { *(.comment) *(.riscv.attributes) }
}
