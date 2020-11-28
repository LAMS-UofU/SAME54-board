#include <RTE_Components.h>
#include "common.h"
#include "drivers.h"

extern volatile uint16_t lidar_timer;
extern volatile uint8_t status;

volatile uint32_t systick_count = 0;

void HardFault_Handler(void) __attribute__( ( naked ) );
void HardFault_Handler_C(unsigned * svc_args, unsigned int lr_value);

/**
  *	HardFAult Handler wrapper in assembly language. Extracts location of stack frame
  * and passes it to the handler written in C as a pointer. Also extracts the LR
  * value as the second parameter
  */
void HardFault_Handler(void)
{
	__asm volatile
	(
		" TST	LR, #4				\n"
		" ITE	EQ					\n"
		" MRSEQ	R0, MSP				\n"
		" MRSNE	R0, PSP				\n"
		" MOV	R1, LR				\n"
		" B		HardFault_Handler_C	\n"
	);
}

/**
  * HardFault Handler in C, with stack frame location and LR value extracted from
  * assembly wrapper as input parameters. Same for all Cortex-M4 Processors.
  */
void HardFault_Handler_C(unsigned int * hardfault_args, unsigned int lr_value)
{	
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;
	uint32_t cfsr;
	uint32_t bus_fault_address;
	uint32_t memmanage_fault_address;
	
	bus_fault_address       = SCB->BFAR;
	memmanage_fault_address = SCB->MMFAR;
	cfsr                    = SCB->CFSR;
	
	stacked_r0  = ((unsigned int) hardfault_args[0]);
	stacked_r1  = ((unsigned int) hardfault_args[1]);
	stacked_r2  = ((unsigned int) hardfault_args[2]);
	stacked_r3  = ((unsigned int) hardfault_args[3]);
	stacked_r12 = ((unsigned int) hardfault_args[4]);
	stacked_lr  = ((unsigned int) hardfault_args[5]);
	stacked_pc  = ((unsigned int) hardfault_args[6]);
	stacked_psr = ((unsigned int) hardfault_args[7]);
	
	printf("[HardFault]\r\n");
	printf(" | Stack frame:\r\n");
	printf(" | | R0   = 0x%08X\r\n", (unsigned)stacked_r0);
	printf(" | | R1   = 0x%08X\r\n", (unsigned)stacked_r1);
	printf(" | | R2   = 0x%08X\r\n", (unsigned)stacked_r2);
	printf(" | | R3   = 0x%08X\r\n", (unsigned)stacked_r3);
	printf(" | | R12  = 0x%08X\r\n", (unsigned)stacked_r12);
	printf(" | | LR   = 0x%08X\r\n", (unsigned)stacked_lr);
	printf(" | | PC   = 0x%08X\r\n", (unsigned)stacked_pc);
	printf(" | | PSR  = 0x%08X\r\n", (unsigned)stacked_psr);
	printf(" | FSR/FAR:\r\n");
	
	printf(" | | Configurable Fault Status Register\r\n");
	printf(" | | | CFSR  = 0x%08X\r\n", (unsigned)cfsr);
	printf(" | | | | MemManage Status Register\r\n");
	printf(" | | | | | MMFSR = 0x%02X\r\n", (unsigned)(cfsr & 0xFF));
	printf(" | | | | BusFault Status Register\r\n");
	printf(" | | | | | BFSR = 0x%02X\r\n", (unsigned)((cfsr >> 8) & 0xFF));
	printf(" | | | | UsageFault Status Register\r\n");
	printf(" | | | | | UFSR = 0x%02X\r\n", (unsigned)((cfsr >> 16) & 0xFF));
	
	printf(" | | MemManage Address Register (validity: %X)\r\n", (unsigned)((cfsr >> 7) & 0x1));
	printf(" | | | MMFAR = 0x%08X\r\n", (unsigned)memmanage_fault_address);
	printf(" | | BusFault Address Register  (validity: %X)\r\n", (unsigned)((cfsr >> 15) & 0x1));
	printf(" | | | BFAR = 0x%08X\r\n", (unsigned)bus_fault_address);
	
	printf(" | | Hard Fault Status Register\r\n");
	printf(" | | | HFSR  = 0x%08X\r\n", (unsigned)SCB->HFSR);
	
	printf(" | | DFSR = 0x%08X\r\n", (unsigned)SCB->DFSR);
	printf(" | | AFSR = 0x%08X\r\n", (unsigned)SCB->AFSR);

	printf(" | Misc\r\n");
	printf(" | | LR/EXC_RETURN= 0x%04X\r\n", (unsigned)lr_value);
	
	while (1) {
		gpio_toggle_pin_level(LED_STATUS);
		delay_ms(BLINK_ERROR);
	}
}
