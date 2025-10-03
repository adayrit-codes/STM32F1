// Basic GPIO helpers for STM32F1
#ifndef GPIO_H
#define GPIO_H

#include "stm32f1.h"
#include "RCC.h"
#include <stdint.h>
#include <stddef.h>

// Pin helpers
#define GPIO_PIN_MASK(pin) (1U << (pin))

// Output speeds
#define GPIO_SPEED_10 1U
#define GPIO_SPEED_2  2U
#define GPIO_SPEED_50 3U

// Return codes
#define GPIO_OK 0
#define GPIO_ERR -1

// Lookup pointer for GPIO port letter 'A'..'G'
static inline GPIO_TypeDef *gpio_port(char port)
{
    switch (port) {
        case 'A': return (GPIO_TypeDef *)GPIOA_BASE;
        case 'B': return (GPIO_TypeDef *)GPIOB_BASE;
        case 'C': return (GPIO_TypeDef *)GPIOC_BASE;
        case 'D': return (GPIO_TypeDef *)GPIOD_BASE;
        case 'E': return (GPIO_TypeDef *)GPIOE_BASE;
        case 'F': return (GPIO_TypeDef *)GPIOF_BASE;
        case 'G': return (GPIO_TypeDef *)GPIOG_BASE;
        default: return NULL;
    }
}

// Enable GPIO clock for port
static inline int gpio_enable_clock(char port)
{
    if (port < 'A' || port > 'G') return GPIO_ERR;
    return rcc_enable_gpio(port) == 0 ? GPIO_OK : GPIO_ERR;
}

// Internal: write 4-bit config for a pin into CRL/CRH
static inline void gpio_set_pin_config(GPIO_TypeDef *gpio, uint32_t pin, uint32_t config)
{
    uint32_t shift = (pin < 8) ? (pin * 4) : ((pin - 8) * 4);
    volatile uint32_t *cr = (pin < 8) ? &gpio->CRL : &gpio->CRH;
    uint32_t tmp = *cr;
    tmp &= ~(0xFu << shift);
    tmp |= ((config & 0xFu) << shift);
    *cr = tmp;
}

// Configure pin as general purpose output
// speed: GPIO_SPEED_10 / GPIO_SPEED_2 / GPIO_SPEED_50
// open_drain: 0 = push-pull, 1 = open-drain
static inline int gpio_config_output(char port, uint32_t pin, uint32_t speed, int open_drain)
{
    if (pin > 15) return GPIO_ERR;
    GPIO_TypeDef *g = gpio_port(port);
    if (!g) return GPIO_ERR;

    // ensure clock enabled
    gpio_enable_clock(port);

    uint32_t mode_bits = (speed & 0x3u); // 1..3
    uint32_t cnf_bits = open_drain ? 0x1u : 0x0u; // for output: CNF=00 push-pull, 01 open-drain
    uint32_t config = (cnf_bits << 2) | (mode_bits & 0x3u);
    gpio_set_pin_config(g, pin, config);
    return GPIO_OK;
}

// Configure pin as input floating
static inline int gpio_config_input_floating(char port, uint32_t pin)
{
    if (pin > 15) return GPIO_ERR;
    GPIO_TypeDef *g = gpio_port(port);
    if (!g) return GPIO_ERR;

    // MODE = 00 (input), CNF = 01 (floating input)
    uint32_t config = (0x1u << 2) | 0x0u;
    gpio_set_pin_config(g, pin, config);
    return GPIO_OK;
}

// Configure pin as input pull-up/pull-down
// pull_up: 1 => pull-up, 0 => pull-down
static inline int gpio_config_input_pull(char port, uint32_t pin, int pull_up)
{
    if (pin > 15) return GPIO_ERR;
    GPIO_TypeDef *g = gpio_port(port);
    if (!g) return GPIO_ERR;

    // MODE = 00 (input), CNF = 10 (pull-up/pull-down)
    uint32_t config = (0x2u << 2) | 0x0u;
    gpio_set_pin_config(g, pin, config);
    if (pull_up)
        g->ODR |= GPIO_PIN_MASK(pin);
    else
        g->ODR &= ~GPIO_PIN_MASK(pin);
    return GPIO_OK;
}

// Configure pin as analog input
static inline int gpio_config_analog(char port, uint32_t pin)
{
    if (pin > 15) return GPIO_ERR;
    GPIO_TypeDef *g = gpio_port(port);
    if (!g) return GPIO_ERR;

    // MODE = 00, CNF = 00 (Analog)
    uint32_t config = 0x0u;
    gpio_set_pin_config(g, pin, config);
    return GPIO_OK;
}

// Read input value (returns 0/1 or -1 on error)
static inline int gpio_read(char port, uint32_t pin)
{
    if (pin > 15) return GPIO_ERR;
    GPIO_TypeDef *g = gpio_port(port);
    if (!g) return GPIO_ERR;
    return ((g->IDR & GPIO_PIN_MASK(pin)) != 0) ? 1 : 0;
}

// Write output value (0 or 1)
static inline int gpio_write(char port, uint32_t pin, int value)
{
    if (pin > 15) return GPIO_ERR;
    GPIO_TypeDef *g = gpio_port(port);
    if (!g) return GPIO_ERR;
    if (value)
        g->BSRR = GPIO_PIN_MASK(pin); // set bit
    else
        g->BRR = GPIO_PIN_MASK(pin);  // reset bit
    return GPIO_OK;
}

// Toggle output pin
static inline int gpio_toggle(char port, uint32_t pin)
{
    if (pin > 15) return GPIO_ERR;
    GPIO_TypeDef *g = gpio_port(port);
    if (!g) return GPIO_ERR;
    g->ODR ^= GPIO_PIN_MASK(pin);
    return GPIO_OK;
}

#endif // GPIO_H
