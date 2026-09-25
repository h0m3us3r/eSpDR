/*
 * Reset entry points for both cores. The ROM bootloader jumps to
 * core0_start after loading the image into RAM; core 0 later releases
 * core 1 into core1_start.
 *
 * Each core: interrupts off, windowed-register ABI reset, coprocessors 0
 * (FPU) and 3 (vector extension) enabled, vectors installed, interrupt
 * level 15 with window overflow exceptions enabled, then its own stack.
 */
    .section .text.startup, "ax"

.macro CORE_SETUP stack_top
    movi a2, 0
    wsr a2, intenable
    wsr a2, windowbase
    movi a2, 1
    wsr a2, windowstart
    movi a2, 9
    wsr a2, cpenable
    movi a2, exception_vectors
    wsr a2, vecbase
    movi a2, 0x4002f
    wsr a2, ps
    rsync
    movi a1, \stack_top
    mov a2, a1
    addi a1, a1, -32
    s32e a2, a1, -12
    movi a0, 0
.endm

    .align 4
    .global core0_start
    .type core0_start, @function
core0_start:
    CORE_SETUP core0_stack_top
    /* Zero .bss. */
    movi a2, _bss_start
    movi a3, _bss_end
    movi a4, 0
1:  bgeu a2, a3, 2f
    s32i a4, a2, 0
    addi a2, a2, 4
    j 1b
2:  call8 app_main
    j fatal_error
    .size core0_start, . - core0_start

    .align 4
    .global core1_start
    .type core1_start, @function
core1_start:
    CORE_SETUP core1_stack_top
    call8 core1_main
    j fatal_error
    .size core1_start, . - core1_start

/* Unrecoverable error: send '!' to the host and stop. */
    .align 4
    .global fatal_error
fatal_error:
    movi a2, 0x60038000 /* USB_SERIAL_JTAG_EP1_REG */
    movi a3, '!'
    s32i a3, a2, 0
    movi a3, 1          /* USB_SERIAL_JTAG_EP1_CONF_REG: WR_DONE */
    s32i a3, a2, 4
3:  j 3b
