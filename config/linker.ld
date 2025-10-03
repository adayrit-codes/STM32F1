
/* Linker script for STM32F1 - 128KB Flash, 20KB RAM
	 Flash start: 0x08000000
	 SRAM start : 0x20000000

	 Provides symbols used by startup.c:
		 _estack, _sidata, _sdata, _edata, _sbss, _ebss
*/

/* Memory layout */
MEMORY
{
	FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
	RAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Default stack size */
_stack_size = 4K;

/* Entry point */
ENTRY(Reset_Handler)

SECTIONS
{
	/* Vector table must be at start of flash */
	.isr_vector :
	{
		KEEP(*(.isr_vector))
	} > FLASH

	.text :
	{
		*(.text*)
		*(.rodata*)
		*(.glue_7)
		*(.glue_7t)
		KEEP(*(.init))
		KEEP(*(.fini))
	} > FLASH

	/* Initialized data goes to RAM, load in FLASH */
	_sidata = LOADADDR(.data);
	.data : AT (ADDR(.text) + SIZEOF(.text))
	{
		_sdata = .;
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM

	/* Uninitialized data */
	.bss (NOLOAD) :
	{
		_sbss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > RAM

	/* Provide end of RAM symbol and stack top */
	PROGBITS_END = ORIGIN(RAM) + LENGTH(RAM);
	_estack = ORIGIN(RAM) + LENGTH(RAM);

	/* Heap (optional) - empty for now */
	.heap (NOLOAD) :
	{
		. = .;
	} > RAM

	/DISCARD/ : { *(.eh_frame) }
}

