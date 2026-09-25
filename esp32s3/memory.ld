/*
 * RAM-only image layout. The ROM bootloader loads every section directly.
 *
 * Internal SRAM is split so that the two cores never contend while streaming:
 *   core 0 code (and everything else)  0x40380000  own instruction bank
 *   core 1 code                         0x40378000  own instruction bank
 *   data, .bss and stacks               0x3FCA0000..0x3FCAFFFF
 *   capture banks 0..3                  0x3FCB0000..0x3FCEFFFF (ADC writer)
 * Each transmit kernel sits at a fixed address at the top of its core's bank.
 * Capture bank 3 overlaps the ROM's working memory; see libc.c.
 */
ENTRY(core0_start)

MEMORY {
    vectors (RX)  : ORIGIN = 0x40374000, LENGTH = 0x4000
    core1   (RX)  : ORIGIN = 0x40378000, LENGTH = 0x8000
    core0   (RX)  : ORIGIN = 0x40380000, LENGTH = 0x10000
    data    (RW)  : ORIGIN = 0x3FCA0000, LENGTH = 0x2000
    work    (RW)  : ORIGIN = 0x3FCA2000, LENGTH = 0xE000
    capture (RW)  : ORIGIN = 0x3FCB0000, LENGTH = 0x40000
}

SECTIONS {
    .vectors : ALIGN(1024) { KEEP(*(.vectors)) } > vectors

    .text : ALIGN(16) {
        *(.literal .literal.* .text .text.*)
        *(.iram1 .iram1.*)
    } > core0

    .core1_text : ALIGN(16) { *(.core1_text .core1_text.*) } > core1

    .tx_lane0 0x4038F000 : { KEEP(*(.tx_lane0)) } > core0
    .tx_lane1 0x4037F000 : { KEEP(*(.tx_lane1)) } > core1

    .data : ALIGN(16) {
        *(.rodata .rodata.* .data .data.* .sdata .sdata.*)
        *(.dram1 .dram1.*)
    } > data

    .bss (NOLOAD) : ALIGN(16) {
        _bss_start = .;
        *(.bss .bss.* COMMON .sbss .sbss.*)
        . = ALIGN(16);
        _bss_end = .;
    } > work

    .stacks (NOLOAD) : ALIGN(16) {
        . += 0x4000;
        core0_stack_top = .;
        . += 0x2000;
        core1_stack_top = .;
    } > work

    .capture (NOLOAD) : { . += 0x40000; } > capture

    /* Debug information: kept in the ELF, never loaded. */
    .debug_info 0 : { *(.debug_info) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_aranges 0 : { *(.debug_aranges) }
    .debug_frame 0 : { *(.debug_frame) }
    .debug_line 0 : { *(.debug_line) }
    .debug_line_str 0 : { *(.debug_line_str) }
    .debug_str 0 : { *(.debug_str) }
    .debug_loc 0 : { *(.debug_loc) }
    .debug_loclists 0 : { *(.debug_loclists) }
    .debug_ranges 0 : { *(.debug_ranges) }
    .debug_rnglists 0 : { *(.debug_rnglists) }

    /DISCARD/ : { *(.comment .note.GNU-stack .eh_frame .xtensa.info .xt.lit .xt.prop .xt.prop.* .xt.lit.*) }
}

ASSERT(ADDR(.text) + SIZEOF(.text) <= 0x4038F000, "core 0 code overlaps its transmit kernel")
ASSERT(ADDR(.core1_text) + SIZEOF(.core1_text) <= 0x4037F000, "core 1 code overlaps its transmit kernel")
ASSERT(SIZEOF(.tx_lane0) > 0 && SIZEOF(.tx_lane1) > 0, "transmit kernels missing")
