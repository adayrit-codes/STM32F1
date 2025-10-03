#include <stdint.h>
// STILL NEED TO ADD ETHERNET AND USB_OTG_FS REGISTERS


// STM32F1 Register Definitions
#ifndef STM32F1_H
#define STM32F1_H

// Base addresss for all Peripherals Registers
#define PERIPH_BASE       0x40000000UL
#define TIM2_BASE        (PERIPH_BASE + 0x00000000UL)
#define TIM3_BASE        (PERIPH_BASE + 0x00000400UL)
#define TIM4_BASE        (PERIPH_BASE + 0x00000800UL)
#define TIM5_BASE        (PERIPH_BASE + 0x00000C00UL)
#define TIM6_BASE        (PERIPH_BASE + 0x00001000UL)
#define TIM7_BASE        (PERIPH_BASE + 0x00001400UL)
#define RTC_BASE         (PERIPH_BASE + 0x00002800UL)
#define WWDG_BASE        (PERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE        (PERIPH_BASE + 0x00003000UL)
#define SPI2_BASE        (PERIPH_BASE + 0x00003800UL)
#define SPI3_BASE        (PERIPH_BASE + 0x00003C00UL)
#define USART2_BASE      (PERIPH_BASE + 0x00004400UL)
#define USART3_BASE      (PERIPH_BASE + 0x00004800UL)
#define UART4_BASE       (PERIPH_BASE + 0x00004C00UL)
#define UART5_BASE       (PERIPH_BASE + 0x00005000UL)
#define I2C1_BASE        (PERIPH_BASE + 0x00005400UL)
#define I2C2_BASE        (PERIPH_BASE + 0x00005800UL)
#define USB_FS_BASE      (PERIPH_BASE + 0x00005C00UL)
#define CAN_BASE         (PERIPH_BASE + 0x00006000UL)
#define BXCAN1_BASE      (PERIPH_BASE + 0x00006400UL)
#define BXCAN2_BASE      (PERIPH_BASE + 0x00006800UL)
#define BKP_BASE         (PERIPH_BASE + 0x00006C00UL)
#define PWR_BASE         (PERIPH_BASE + 0x00007000UL)
#define DAC_BASE         (PERIPH_BASE + 0x00007400UL)
#define AFIO_BASE        (PERIPH_BASE + 0x00010000UL)
#define EXTI_BASE        (PERIPH_BASE + 0x00010400UL)
#define GPIOA_BASE       (PERIPH_BASE + 0x00010800UL)
#define GPIOB_BASE       (PERIPH_BASE + 0x00010C00UL)
#define GPIOC_BASE       (PERIPH_BASE + 0x00011000UL)
#define GPIOD_BASE       (PERIPH_BASE + 0x00011400UL)
#define GPIOE_BASE       (PERIPH_BASE + 0x00011800UL)
#define GPIOF_BASE       (PERIPH_BASE + 0x00011C00UL)
#define GPIOG_BASE       (PERIPH_BASE + 0x00012000UL)
#define ADC1_BASE        (PERIPH_BASE + 0x00012400UL)
#define ADC2_BASE        (PERIPH_BASE + 0x00012800UL)
#define TIM1_BASE        (PERIPH_BASE + 0x00012C00UL)
#define SPI1_BASE        (PERIPH_BASE + 0x00013000UL)
#define USART1_BASE      (PERIPH_BASE + 0x00013800UL)
#define ADC3_BASE        (PERIPH_BASE + 0x00013C00UL)
#define SDIO_BASE        (PERIPH_BASE + 0x00018000UL)
#define DMA1_BASE        (PERIPH_BASE + 0x00020000UL)
#define DMA2_BASE        (PERIPH_BASE + 0x00020400UL)
#define RCC_BASE         (PERIPH_BASE + 0x00021000UL)
#define FLASH_BASE       (PERIPH_BASE + 0x00022000UL)
#define CRC_BASE         (PERIPH_BASE + 0x00023000UL)
#define ETHERNET_MAC_BASE (PERIPH_BASE + 0x00028000UL)
#define USB_OTG_FS_BASE  (PERIPH_BASE + 0x10000000UL)

// Peripheral register structures

// Timer Peripheral Registers
typedef struct {
    volatile uint32_t CR1;       // Control register 1
    volatile uint32_t CR2;       // Control register 2
    volatile uint32_t SMCR;      // Slave mode control register
    volatile uint32_t DIER;      // DMA/Interrupt enable register
    volatile uint32_t SR;        // Status register
    volatile uint32_t EGR;       // Event generation register
    volatile uint32_t CCMR1;     // Input or Output Compare mode
    volatile uint32_t CCMR2;     // Input or Output Compare mode
    volatile uint32_t CCER;      // Capture/Compare enable register
    volatile uint32_t CNT;       // Counter
    volatile uint32_t PSC;       // Prescaler
    volatile uint32_t ARR;       // Auto-reload register
    volatile uint32_t RESEVRED1; // Reserved
    volatile uint32_t CCR1;      // Capture/Compare register 1
    volatile uint32_t CCR2;      // Capture/Compare register 2
    volatile uint32_t CCR3;      // Capture/Compare register 3
    volatile uint32_t CCR4;      // Capture/Compare register 4
    volatile uint32_t RESEVRED2; // Reserved
    volatile uint32_t DCR;       // DMA control register
    volatile uint32_t DMAR;      // DMA address for full transfer
} TIM_TypeDef;

// RTC Peripheral Registers
typedef struct {
    volatile uint32_t CRH;       // Control register high
    volatile uint32_t CRL;       // Control register low
    volatile uint32_t PRLH;      // Prescaler load high
    volatile uint32_t PRLL;      // Prescaler load low
    volatile uint32_t DIVH;      // Divider high
    volatile uint32_t DIVL;      // Divider low
    volatile uint32_t CNTH;      // Counter high
    volatile uint32_t CNTL;      // Counter low
    volatile uint32_t ALRH;      // Alarm high
    volatile uint32_t ALRL;      // Alarm low
} RTC_TypeDef;

// WWDG Peripheral Registers
typedef struct {
    volatile uint32_t CR;        // Control register
    volatile uint32_t CFR;       // Configuration register
    volatile uint32_t SR;        // Status register
} WWDG_TypeDef;

// IWDG Peripheral Registers
typedef struct {
    volatile uint32_t KR;        // Key register
    volatile uint32_t PR;        // Prescaler register
    volatile uint32_t RLR;       // Reload register
    volatile uint32_t SR;        // Status register
} IWDG_TypeDef;

// SPI Peripheral Registers
typedef struct {
    volatile uint32_t CR1;       // Control register 1
    volatile uint32_t CR2;       // Control register 2
    volatile uint32_t SR;        // Status register
    volatile uint32_t DR;        // Data register
    volatile uint32_t CRCPR;     // CRC polynomial register
    volatile uint32_t RXCRCR;    // RX CRC register
    volatile uint32_t TXCRCR;    // TX CRC register
    volatile uint32_t I2SCFGR;   // I2S configuration register
    volatile uint32_t I2SPR;     // I2S prescaler register
} SPI_TypeDef;

// USART Peripheral Registers
typedef struct {
    volatile uint32_t SR;        // Status register
    volatile uint32_t DR;        // Data register
    volatile uint32_t BRR;       // Baud rate register
    volatile uint32_t CR1;       // Control register 1
    volatile uint32_t CR2;       // Control register 2
    volatile uint32_t CR3;       // Control register 3
    volatile uint32_t GTPR;      // Guard time and prescaler register
} USART_TypeDef;

// I2C Peripheral Registers
typedef struct {
    volatile uint32_t CR1;       // Control register 1
    volatile uint32_t CR2;       // Control register 2
    volatile uint32_t OAR1;      // Own address register 1
    volatile uint32_t OAR2;      // Own address register 2
    volatile uint32_t DR;        // Data register
    volatile uint32_t SR1;       // Status Register 1
    volatile uint32_t SR2;       // Status Register 2
    volatile uint32_t CCR;       // Clock control register
    volatile uint32_t TRISE;     // TRISE register
} I2C_TypeDef;

// USB FS Peripheral Registers
typedef struct {
    volatile uint32_t EP0R;      // Endpoint 0 register
    volatile uint32_t EP1R;      // Endpoint 1 register
    volatile uint32_t EP2R;      // Endpoint 2 register
    volatile uint32_t EP3R;      // Endpoint 3 register
    volatile uint32_t EP4R;      // Endpoint 4 register
    volatile uint32_t EP5R;      // Endpoint 5 register
    volatile uint32_t EP6R;      // Endpoint 6 register
    volatile uint32_t EP7R;      // Endpoint 7 register
    volatile uint32_t RESERVED[8]; // Reserved
    volatile uint32_t CNTR;      // Control register
    volatile uint32_t ISTR;      // Interrupt status register
    volatile uint32_t FNR;       // Frame number register
    volatile uint32_t DADDR;     // Device address register
    volatile uint32_t BTABLE;    // Buffer table address register
} USB_FS_TypeDef;

// CAN Peripheral Registers
typedef struct {
    volatile uint32_t MCR;       // Master control register
    volatile uint32_t MSR;       // Master status register
    volatile uint32_t TSR;       // Transmit status register
    volatile uint32_t RF0R;      // Receive FIFO 0 register
    volatile uint32_t RF1R;      // Receive FIFO 1 register
    volatile uint32_t IER;       // Interrupt enable register
    volatile uint32_t ESR;       // Error status register
    volatile uint32_t BTR;       // Bit timing register
    volatile uint32_t RESERVED[88]; // Reserved
    volatile uint32_t TI0R;      // Transmit identifier register
    volatile uint32_t TDT0R;     // Transmit data length control and time
    volatile uint32_t TDL0R;     // Transmit data low register
    volatile uint32_t TDH0R;     // Transmit data high register
    volatile uint32_t TI1R;      // Transmit identifier register
    volatile uint32_t TDT1R;     // Transmit data length control and time
    volatile uint32_t TDL1R;     // Transmit data low register
    volatile uint32_t TDH1R;     // Transmit data high register
    volatile uint32_t TI2R;      // Transmit identifier register
    volatile uint32_t TDT2R;     // Transmit data length control and time
    volatile uint32_t TDL2R;     // Transmit data low register
    volatile uint32_t TDH2R;     // Transmit data high register
    volatile uint32_t RI0R;      // Receive identifier register
    volatile uint32_t RDT0R;     // Receive data length control and time
    volatile uint32_t RDL0R;     // Receive data low register
    volatile uint32_t RDH0R;     // Receive data high register
    volatile uint32_t RI1R;      // Receive identifier register
    volatile uint32_t RDT1R;     // Receive data length control and time
    volatile uint32_t RDL1R;     // Receive data low register
    volatile uint32_t RDH1R;     // Receive data high register
    volatile uint32_t RESERVED1[12]; // Reserved
    volatile uint32_t FMR;       // Filter master register
    volatile uint32_t FM1R;      // Filter mode register
    volatile uint32_t RESERVED2; // Reserved
    volatile uint32_t FS1R;      // Filter scale register
    volatile uint32_t RESERVED3; // Reserved
    volatile uint32_t FFA1R;     // Filter FIFO assignment register
    volatile uint32_t RESERVED4; // Reserved
    volatile uint32_t FA1R;      // Filter activation register
    volatile uint32_t RESERVED5[8]; // Reserved
    volatile uint32_t F0R1;     // Filter bank 0 register 1
    volatile uint32_t F0R2;     // Filter bank 0 register 2
    volatile uint32_t F1R1;     // Filter bank 1 register 1
    volatile uint32_t F1R2;     // Filter bank 1 register 2
    volatile uint32_t RESERVED6[51]; // Reserved
    volatile uint32_t F27R1;    // Filter bank 27 register 1
    volatile uint32_t F27R2;    // Filter bank 27 register 2
} CAN_TypeDef;

// Backup Registers
typedef struct {
    volatile uint32_t RESERVED1; // Reserved
    volatile uint32_t DR1;      // Data register 1
    volatile uint32_t DR2;      // Data register 2
    volatile uint32_t DR3;      // Data register 3
    volatile uint32_t DR4;      // Data register 4
    volatile uint32_t DR5;      // Data register 5
    volatile uint32_t DR6;      // Data register 6
    volatile uint32_t DR7;      // Data register 7
    volatile uint32_t DR8;      // Data register 8
    volatile uint32_t DR9;      // Data register 9
    volatile uint32_t DR10;     // Data register 10
    volatile uint32_t RTCCR;    // RTC clock calibration register
    volatile uint32_t CR;       // Control register
    volatile uint32_t CSR;      // Control/status register
    volatile uint32_t RESERVED2[2]; // Reserved
    volatile uint32_t DR11;     // Data register 11
    volatile uint32_t DR12;     // Data register 12
    volatile uint32_t DR13;     // Data register 13
    volatile uint32_t DR14;     // Data register 14
    volatile uint32_t DR15;     // Data register 15
    volatile uint32_t DR16;     // Data register 16
    volatile uint32_t DR17;     // Data register 17
    volatile uint32_t DR18;     // Data register 18
    volatile uint32_t DR19;     // Data register 19
    volatile uint32_t DR20;     // Data register 20
    volatile uint32_t DR21;     // Data register 21
    volatile uint32_t DR22;     // Data register 22
    volatile uint32_t DR23;     // Data register 23
    volatile uint32_t DR24;     // Data register 24
    volatile uint32_t DR25;     // Data register 25
    volatile uint32_t DR26;     // Data register 26
    volatile uint32_t DR27;     // Data register 27
    volatile uint32_t DR28;     // Data register 28
    volatile uint32_t DR29;     // Data register 29
    volatile uint32_t DR30;     // Data register 30
    volatile uint32_t DR31;     // Data register 31
    volatile uint32_t DR32;     // Data register 32
    volatile uint32_t DR33;     // Data register 33
    volatile uint32_t DR34;     // Data register 34
    volatile uint32_t DR35;     // Data register 35
    volatile uint32_t DR36;     // Data register 36
    volatile uint32_t DR37;     // Data register 37
    volatile uint32_t DR38;     // Data register 38
    volatile uint32_t DR39;     // Data register 39
    volatile uint32_t DR40;     // Data register 40
    volatile uint32_t DR41;     // Data register 41
    volatile uint32_t DR42;     // Data register 42
} BKP_TypeDef;

// Power Control Registers
typedef struct {
    volatile uint32_t CR;       // Power control register
    volatile uint32_t CSR;      // Power control/status register
} PWR_TypeDef;

// DAC Peripheral Registers
typedef struct {
    volatile uint32_t CR;       // Control register
    volatile uint32_t SWTRIGR;  // Software trigger register
    volatile uint32_t DHR12R1;  // Channel 1 12-bit
    volatile uint32_t DHR12L1;  // Channel 1 12-bit left aligned
    volatile uint32_t DHR8R1;   // Channel 1 8-bit
    volatile uint32_t DHR12R2;  // Channel 2 12-bit
    volatile uint32_t DHR12L2;  // Channel 2 12-bit left aligned
    volatile uint32_t DHR8R2;   // Channel 2
    volatile uint32_t DHR12RD;  // Dual 12-bit right aligned
    volatile uint32_t DHR12LD;  // Dual 12-bit left aligned
    volatile uint32_t DHR8RD;   // Dual 8-bit right aligned
    volatile uint32_t DOR1;     // Channel 1 data output register
    volatile uint32_t DOR2;     // Channel 2 data output register
} DAC_TypeDef;

// AFIO Peripheral Registers
typedef struct {
    volatile uint32_t EVCR;     // Event control register
    volatile uint32_t MAPR;     // AF remap and debug I/O configuration register
    volatile uint32_t EXTICR[4]; // External interrupt configuration registers
} AFIO_TypeDef;

// EXTI Peripheral Registers
typedef struct {
    volatile uint32_t IMR;      // Interrupt mask register
    volatile uint32_t EMR;      // Event mask register
    volatile uint32_t RTSR;     // Rising trigger selection register
    volatile uint32_t FTSR;     // Falling trigger selection register
    volatile uint32_t SWIER;    // Software interrupt event register
    volatile uint32_t PR;       // Pending Register
} EXTI_Typedef;

// GPIO Peripheral Registers
typedef struct {
    volatile uint32_t CRL;      // Port configuration register low
    volatile uint32_t CRH;      // Port configuration register high
    volatile uint32_t IDR;      // Port input data register
    volatile uint32_t ODR;      // Port output data register
    volatile uint32_t BSRR;     // Port bit set/reset register
    volatile uint32_t BRR;
    volatile uint32_t LCKR;     // Port configuration lock register
} GPIO_TypeDef;

// ADC Peripheral Registers
typedef struct {
    volatile uint32_t SR;       // Status register
    volatile uint32_t CR1;      // Control register 1
    volatile uint32_t CR2;      // Control register 2
    volatile uint32_t SMPR1;    // Sample time register 1
    volatile uint32_t SMPR2;    // Sample time register 2
    volatile uint32_t JOFR1;    // Injected channel data offset register 1
    volatile uint32_t JOFR2;    // Injected channel data offset register 2
    volatile uint32_t JOFR3;    // Injected channel data offset register 3
    volatile uint32_t JOFR4;    // Injected channel data offset register 4
    volatile uint32_t HTR;      // Watchdog higher threshold register
    volatile uint32_t LTR;      // Watchdog lower threshold register
    volatile uint32_t SQR1;     // Regular sequence register 1
    volatile uint32_t SQR2;     // Regular sequence register 2
    volatile uint32_t SQR3;     // Regular sequence register 3
    volatile uint32_t JSQR;     // Injected sequence register
    volatile uint32_t JDR1;     // Injected data register 1
    volatile uint32_t JDR2;     // Injected data register 2
    volatile uint32_t JDR3;     // Injected data register 3
    volatile uint32_t JDR4;     // Injected data register 4
    volatile uint32_t DR;       // Regular data register
} ADC_TypeDef;

// SDIO Peripheral Registers
typedef struct {
    volatile uint32_t POWER;    // Power control register
    volatile uint32_t CLKCR;    // Clock control register
    volatile uint32_t ARG;      // Argument register
    volatile uint32_t CMD;      // Command register
    volatile uint32_t RESPCMD;  // Response command register
    volatile uint32_t RESP1;    // Response 1 register
    volatile uint32_t RESP2;    // Response 2 register
    volatile uint32_t RESP3;    // Response 3 register
    volatile uint32_t RESP4;    // Response 4 register
    volatile uint32_t DTIMER;   // Data timer register
    volatile uint32_t DLEN;     // Data length register
    volatile uint32_t DCTRL;    // Data control register
    volatile uint32_t DCOUNT;   // Data counter register
    volatile uint32_t STA;      // Status register
    volatile uint32_t ICR;      // Interrupt clear register
    volatile uint32_t MASK;     // Mask register
    volatile uint32_t RESERVED[2]; // Reserved
    volatile uint32_t FIFOCNT;  // FIFO counter register
    volatile uint32_t RESERVED1[13]; // Reserved
    volatile uint32_t FIFO;     // Data FIFO register
} SDIO_TypeDef;

// DMA Peripheral Registers
typedef struct {
    volatile uint32_t ISR;      // Interrupt status register
    volatile uint32_t IFCR;     // Interrupt flag clear register
    volatile uint32_t CCR1;   // Channel 1 configuration register
    volatile uint32_t CNDTR1; // Channel 1 number of data register
    volatile uint32_t CPAR1;  // Channel 1 peripheral address register
    volatile uint32_t CMAR1;  // Channel 1 memory address register
    volatile uint32_t RESERVED1; // Reserved
    volatile uint32_t CCR2;   // Channel 2 configuration register
    volatile uint32_t CNDTR2; // Channel 2 number of data register
    volatile uint32_t CPAR2;  // Channel 2 peripheral address register
    volatile uint32_t CMAR2;  // Channel 2 memory address register
    volatile uint32_t RESERVED2; // Reserved
    volatile uint32_t CCR3;   // Channel 3 configuration register
    volatile uint32_t CNDTR3; // Channel 3 number of data register
    volatile uint32_t CPAR3;  // Channel 3 peripheral address register
    volatile uint32_t CMAR3;  // Channel 3 memory address register
    volatile uint32_t RESERVED3; // Reserved
    volatile uint32_t CCR4;   // Channel 4 configuration register
    volatile uint32_t CNDTR4; // Channel 4 number of data register
    volatile uint32_t CPAR4;  // Channel 4 peripheral address register
    volatile uint32_t CMAR4;  // Channel 4 memory address register
    volatile uint32_t RESERVED4; // Reserved
    volatile uint32_t CCR5;   // Channel 5 configuration register
    volatile uint32_t CNDTR5; // Channel 5 number of data register
    volatile uint32_t CPAR5;  // Channel 5 peripheral address register
    volatile uint32_t CMAR5;  // Channel 5 memory address register
    volatile uint32_t RESERVED5; // Reserved
    volatile uint32_t CCR6;   // Channel 6 configuration register
    volatile uint32_t CNDTR6; // Channel 6 number of data register
    volatile uint32_t CPAR6;  // Channel 6 peripheral address register
    volatile uint32_t CMAR6;  // Channel 6 memory address register
    volatile uint32_t RESERVED6; // Reserved
    volatile uint32_t CCR7;   // Channel 7 configuration register
    volatile uint32_t CNDTR7; // Channel 7 number of data register
    volatile uint32_t CPAR7;  // Channel 7 peripheral address register
    volatile uint32_t CMAR7;  // Channel 7 memory address register
    volatile uint32_t RESERVED7; // Reserved
} DMA_TypeDef;

// RCC Peripheral Registers
typedef struct {
    volatile uint32_t CR;       // Clock control register
    volatile uint32_t CFGR;     // Clock configuration register
    volatile uint32_t CIR;      // Clock interrupt register
    volatile uint32_t APB2RSTR; // APB2 peripheral reset register
    volatile uint32_t APB1RSTR; // APB1 peripheral reset register
    volatile uint32_t AHBENR; 
    volatile uint32_t APB2ENR;  // APB2 peripheral clock enable register
    volatile uint32_t APB1ENR;  // APB1 peripheral clock enable
    volatile uint32_t BDCR;     // Backup domain control register
    volatile uint32_t CSR;      // Control/status register
} RCC_TypeDef;

// CRC Peripheral Registers
typedef struct {
    volatile uint32_t DR;       // Data register
    volatile uint32_t IDR;      // Independent data register
    volatile uint32_t CR;       // Control register
} CRC_TypeDef;

// STILL NEED TO ADD ETHERNET AND USB_OTG_FS REGISTERS

#endif // STM32F1_H