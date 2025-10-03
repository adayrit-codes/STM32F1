// RCC Peripheral Register Functions
#ifndef RCC_H
#define RCC_H

#include "stm32f1.h"
#include <stdint.h>

// Convenience pointer to the RCC registers
#define RCC_REG ((RCC_TypeDef *)RCC_BASE)

// Generic helpers: set/clear mask bits in the APB2/APB1/AHB enable registers.
static inline void rcc_enable_apb2(uint32_t mask)
{
	RCC_REG->APB2ENR |= mask;
	__asm volatile ("dsb\n\t"); // data synchronization barrier
}

static inline void rcc_disable_apb2(uint32_t mask)
{
	RCC_REG->APB2ENR &= ~mask;
	__asm volatile ("dsb\n\t");
}

static inline void rcc_enable_apb1(uint32_t mask)
{
	RCC_REG->APB1ENR |= mask;
	__asm volatile ("dsb\n\t");
}

static inline void rcc_disable_apb1(uint32_t mask)
{
	RCC_REG->APB1ENR &= ~mask;
	__asm volatile ("dsb\n\t");
}

static inline void rcc_enable_ahb(uint32_t mask)
{
	RCC_REG->AHBENR |= mask;
	__asm volatile ("dsb\n\t");
}

static inline void rcc_disable_ahb(uint32_t mask)
{
	RCC_REG->AHBENR &= ~mask;
	__asm volatile ("dsb\n\t");
}

// Common APB2 bit positions (reference manual ordering):
// AFIO: bit 0, GPIOA: bit 2, GPIOB: bit 3, GPIOC: bit 4, GPIOD: bit 5,
// GPIOE: bit 6, GPIOF: bit 7, GPIOG: bit 8
#define RCC_APB2_AFIOEN   (1U << 0)
#define RCC_APB2_IOPAEN   (1U << 2)
#define RCC_APB2_IOPBEN   (1U << 3)
#define RCC_APB2_IOPCEN   (1U << 4)
#define RCC_APB2_IOPDEN   (1U << 5)
#define RCC_APB2_IOPEEN   (1U << 6)
#define RCC_APB2_IOPFEN   (1U << 7)
#define RCC_APB2_IOPGEN   (1U << 8)

// Convenience wrappers for AFIO and GPIO port clocks
static inline void rcc_enable_afio(void)
{
	rcc_enable_apb2(RCC_APB2_AFIOEN);
}

static inline void rcc_disable_afio(void)
{
	rcc_disable_apb2(RCC_APB2_AFIOEN);
}

// Enable/disable GPIO port by letter 'A'..'G'. Returns 0 on success, -1 on invalid port.
static inline int rcc_enable_gpio(char port)
{
	if (port < 'A' || port > 'G') return -1;
	uint32_t bit = (uint32_t)(2 + (port - 'A'));
	rcc_enable_apb2(1U << bit);
	return 0;
}

static inline int rcc_disable_gpio(char port)
{
	if (port < 'A' || port > 'G') return -1;
	uint32_t bit = (uint32_t)(2 + (port - 'A'));
	rcc_disable_apb2(1U << bit);
	return 0;
}

#endif // RCC_H

