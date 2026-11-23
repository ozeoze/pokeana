/*
 * EBI test traffic for pokeana's parallel bus decoder, on the NuMaker-Server-NUC980
 * (NK-980ETH2P V1.2). CON23 carries D0-D15, A0-A3, nCS1, nWE and nRE.
 *
 * Each cycle n writes 16 words to bank 1 (word k at A[3:0] = k), then reads the 16 back,
 * then waits about 1 ms. The words say which cycle they belong to, so a capture can be
 * checked on its own (tests/nuc980_ebi.py has the same formula):
 *   k = 0: n & 0xFFFF     k = 1: n >> 13     k = 2: 1 << (n & 15)    k = 3: ~(1 << (n & 15))
 * (D13-D15 can't follow on this board: the keypad's 0.1 uF on PC13-15, so n is in 13-bit pieces)
 *   k >= 4: (x * 1103515245 + 12345) >> 16 with x = n * 16 + k (32-bit arithmetic)
 * Nothing answers the reads (no device on the bus), so their data is whatever floats.
 *
 * UART0 (CON5) prints the cycle count every 1000 cycles; 'R' resets the chip back to
 * USB boot. Register definitions as in i386_sbc's prestudy/firmware/ebitest, from
 * OpenNuvoton/NUC980_NonOS_BSP and the NUC980 datasheet Rev 1.24.
 */
#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

#define SYS_AHBIPRST 0xB0000060u
#define SYS_GPA_MFPL 0xB0000070u
#define SYS_GPA_MFPH 0xB0000074u
#define SYS_GPB_MFPL 0xB0000078u
#define SYS_GPC_MFPL 0xB0000080u
#define SYS_GPC_MFPH 0xB0000084u
#define SYS_GPF_MFPH 0xB000009Cu
#define SYS_GPG_MFPL 0xB00000A0u
#define SYS_REGWPCTL 0xB00001FCu
#define CLK_HCLKEN 0xB0000210u
#define CLK_PCLKEN0 0xB0000218u
#define CLK_HCLKEN_EBI (1u << 9)
#define CLK_HCLKEN_GPIO (1u << 11)
#define CLK_PCLKEN0_UART0 (1u << 16)

#define UART0_DAT 0xB0070000u
#define UART0_LINE 0xB007000Cu
#define UART0_FIFOSTS 0xB0070018u
#define UART0_BAUD 0xB0070024u
#define UART_RXEMPTY (1u << 14)
#define UART_TXFULL (1u << 23)
#define UART_TXEMPTY_DONE (1u << 28)

#define EBI_CTL1 0xB0010010u
#define EBI_TCTL1 0xB0010014u
#define EBI_CTL_EN (1u << 0)
#define EBI_CTL_DW16 (1u << 1)
#define EBI_TCTL_TACC_POS 3u
#ifndef EBI_TACC
#define EBI_TACC 31u /* slowest: wide strobes for a logic analyser */
#endif

#define EBI_BANK1 ((volatile uint16_t *)0x60100000u)
#define MFP_EBI 1u
#define WORDS 16u
#define GAP_DELAY 150000u

static void delay(uint32_t n)
{
    while (n--)
        __asm__ volatile("nop");
}

static void mfp_set(uint32_t reg, uint32_t bit, uint32_t func)
{
    uint32_t shift = (bit & 7u) * 4u;

    REG(reg) = (REG(reg) & ~(0xFu << shift)) | (func << shift);
}

static void uart_init(void)
{
    REG(CLK_PCLKEN0) |= CLK_PCLKEN0_UART0;
    REG(SYS_GPF_MFPH) = (REG(SYS_GPF_MFPH) & 0xFFF00FFFu) | 0x00011000u;
    REG(UART0_LINE) |= 0x07u;
    REG(UART0_BAUD) = 0x30000066u; /* 115200 from the 12 MHz reference */
}

static void uart_putc(char c)
{
    while (REG(UART0_FIFOSTS) & UART_TXFULL)
        ;
    REG(UART0_DAT) = (uint32_t)(uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

static void uart_putdec(uint32_t v)
{
    char buf[11];
    int i = 0;

    do {
        buf[i++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v);
    while (i)
        uart_putc(buf[--i]);
}

static void ebi_init(void)
{
    REG(CLK_HCLKEN) |= CLK_HCLKEN_EBI | CLK_HCLKEN_GPIO;

    mfp_set(SYS_GPA_MFPL, 6, MFP_EBI); /* nCS1 */
    mfp_set(SYS_GPA_MFPL, 7, MFP_EBI); /* nWE */
    mfp_set(SYS_GPA_MFPH, 8, MFP_EBI); /* nRE */
    mfp_set(SYS_GPG_MFPL, 0, MFP_EBI); /* ADDR0 */
    mfp_set(SYS_GPG_MFPL, 1, MFP_EBI); /* ADDR1 */
    mfp_set(SYS_GPB_MFPL, 2, MFP_EBI); /* ADDR2 (does not move on this board) */
    mfp_set(SYS_GPG_MFPL, 3, MFP_EBI); /* ADDR3 */
    for (uint32_t bit = 0; bit < 8; bit++) {
        mfp_set(SYS_GPC_MFPL, bit, MFP_EBI); /* DATA0-7 */
        mfp_set(SYS_GPC_MFPH, bit, MFP_EBI); /* DATA8-15 */
    }

    /* 16-bit, MCLK = HCLK, longest access time */
    REG(EBI_CTL1) = EBI_CTL_DW16 | EBI_CTL_EN;
    REG(EBI_TCTL1) = EBI_TACC << EBI_TCTL_TACC_POS;
}

static void chip_reset(void)
{
    uart_puts("ebi_pattern: chip reset\r\n");
    while (!(REG(UART0_FIFOSTS) & UART_TXEMPTY_DONE))
        ;
    do {
        REG(SYS_REGWPCTL) = 0x59u;
        REG(SYS_REGWPCTL) = 0x16u;
        REG(SYS_REGWPCTL) = 0x88u;
    } while (REG(SYS_REGWPCTL) == 0u);
    REG(SYS_AHBIPRST) |= 1u;
    for (;;)
        ;
}

static uint16_t word(uint32_t n, uint32_t k)
{
    uint32_t bit = 1u << (n & 15u);

    switch (k) {
    case 0:
        return (uint16_t)n;
    case 1:
        return (uint16_t)(n >> 13);
    case 2:
        return (uint16_t)bit;
    case 3:
        return (uint16_t)~bit;
    default:
        return (uint16_t)(((n * 16u + k) * 1103515245u + 12345u) >> 16);
    }
}

int main(void)
{
    volatile uint16_t *bank = EBI_BANK1;
    uint16_t buf[WORDS];

    uart_init();
    uart_puts("\r\nebi_pattern: TACC=");
    uart_putdec(EBI_TACC);
    uart_puts(", send R to reset\r\n");
    ebi_init();

    for (uint32_t n = 0;; n++) {
        for (uint32_t k = 0; k < WORDS; k++)
            buf[k] = word(n, k);
        for (uint32_t k = 0; k < WORDS; k++)
            bank[k] = buf[k];
        for (uint32_t k = 0; k < WORDS; k++)
            (void)bank[k];
        delay(GAP_DELAY);

        if (n % 1000u == 0u) {
            uart_puts("cycle ");
            uart_putdec(n);
            uart_puts("\r\n");
        }
        if (!(REG(UART0_FIFOSTS) & UART_RXEMPTY) && (REG(UART0_DAT) & 0xFFu) == 'R')
            chip_reset();
    }
}
