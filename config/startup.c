/* Minimal startup for STM32F1: vector table, reset handler, and default handlers */
#include <stdint.h>

/* Linker-provided symbols (must be defined in the project's linker script) */
extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

/* Forward declaration of the main application entry point */
int main(void);

/* Exception / IRQ handlers */
void Reset_Handler(void);
void NMI_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias("Default_Handler")));

void Default_Handler(void)
{
    /* Infinite loop: a real project might reset or log */
    while (1) {
        __asm__ volatile ("wfi");
    }
}

/* Vector table placed in the .isr_vector section. First entry is the initial
   stack pointer value; subsequent entries are handler addresses. Using a
   uint32_t array avoids casting the stack pointer to a function pointer. */
__attribute__ ((section(".isr_vector"), used, aligned(8)))
const void * const vector_table[] = {
    (const void *) &_estack,        /* Initial Stack Pointer */
    (const void *) Reset_Handler,   /* Reset Handler */
    (const void *) NMI_Handler,     /* NMI Handler */
    (const void *) HardFault_Handler,/* Hard Fault Handler */
    (const void *) MemManage_Handler,/* MPU Fault Handler */
    (const void *) BusFault_Handler, /* Bus Fault Handler */
    (const void *) UsageFault_Handler,/* Usage Fault Handler */
    0, 0, 0, 0,                  /* Reserved */
    (const void *) SVC_Handler,      /* SVCall Handler */
    (const void *) DebugMon_Handler, /* Debug Monitor */
    0,                           /* Reserved */
    (const void *) PendSV_Handler,   /* PendSV */
    (const void *) SysTick_Handler,  /* SysTick */
    /* Device IRQs: map to Default_Handler by default */
    (const void *) Default_Handler, /* IRQ0 */
    (const void *) Default_Handler, /* IRQ1 */
    (const void *) Default_Handler, /* IRQ2 */
    (const void *) Default_Handler, /* IRQ3 */
    (const void *) Default_Handler, /* IRQ4 */
    (const void *) Default_Handler, /* IRQ5 */
    (const void *) Default_Handler, /* IRQ6 */
    (const void *) Default_Handler, /* IRQ7 */
    (const void *) Default_Handler, /* IRQ8 */
    (const void *) Default_Handler, /* IRQ9 */
    (const void *) Default_Handler, /* IRQ10 */
    (const void *) Default_Handler, /* IRQ11 */
    (const void *) Default_Handler, /* IRQ12 */
    (const void *) Default_Handler, /* IRQ13 */
    (const void *) Default_Handler, /* IRQ14 */
    (const void *) Default_Handler, /* IRQ15 */
    (const void *) Default_Handler, /* IRQ16 */
    (const void *) Default_Handler, /* IRQ17 */
    (const void *) Default_Handler, /* IRQ18 */
    (const void *) Default_Handler, /* IRQ19 */
    (const void *) Default_Handler, /* IRQ20 */
    (const void *) Default_Handler, /* IRQ21 */
    (const void *) Default_Handler, /* IRQ22 */
    (const void *) Default_Handler, /* IRQ23 */
    (const void *) Default_Handler, /* IRQ24 */
    (const void *) Default_Handler, /* IRQ25 */
    (const void *) Default_Handler, /* IRQ26 */
    (const void *) Default_Handler, /* IRQ27 */
    (const void *) Default_Handler, /* IRQ28 */
    (const void *) Default_Handler, /* IRQ29 */
    (const void *) Default_Handler, /* IRQ30 */
    (const void *) Default_Handler, /* IRQ31 */
};

/* Reset handler: initialize .data and .bss, then call main() */
void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    /* Copy .data section from flash to SRAM */
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    /* Zero initialize .bss */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0U;
    }

    /* Call application's entry point */
    (void) main();

    /* If main returns, loop forever */
    while (1) {
        __asm__ volatile ("wfi");
    }
}
