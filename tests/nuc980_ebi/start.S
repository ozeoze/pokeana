/* Entry point for code loaded into DDR by NuWriter. */
    .section .init, "ax"
    .arm
    .global _start
_start:
    msr     cpsr_c, #0xD3       /* SVC mode, IRQ and FIQ masked */
    ldr     sp, =__stack_top

    ldr     r0, =__bss_start
    ldr     r1, =__bss_end
    mov     r2, #0
1:  cmp     r0, r1
    strlo   r2, [r0], #4
    blo     1b

    bl      main
2:  b       2b
