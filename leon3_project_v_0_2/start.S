.section .text
.global _start

_start:
    ! init Stack Pointer (%sp) to 64KB
    set 0x40010000, %sp
    
    ! init Frame Pointer (%fp)
    mov %sp, %fp

    ! clean wim
    mov 2, %g1
    mov %g1, %wim

    call main
    nop

loop:
    ba loop
    nop
