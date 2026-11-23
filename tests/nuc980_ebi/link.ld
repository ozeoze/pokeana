ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x00008000, LENGTH = 16M
}

SECTIONS
{
    .text : {
        KEEP(*(.init))
        *(.text*)
        *(.rodata*)
    } > RAM

    .data : { *(.data*) } > RAM

    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    . = ALIGN(8);
    . = . + 0x4000;
    __stack_top = .;

    /DISCARD/ : { *(.comment) *(.ARM.*) }
}
