#include "start.h"
#include "eeprom.h"

/* A specific byte pattern stored at the beginning of SmartEEPROM data area.
 * When the application comes from a reset, if it finds this signature,
 * the assumption is that the SmartEEPROM has some valid data.
 */
#define SMEE_CUSTOM_SIG 0x5a5a5a5a

/* Define the number of bytes to be read when a read request comes */
#define TEST_BUFF_SIZE 100

/* The application toggles SmartEEPROM data at the following address
 * each time the device is reset. This address must be within the total
 * number of EEPROM data bytes ( Defined by SBLK and PSZ fuse values).
 */
#define SEEP_TEST_ADDR 32

/* This example assumes SBLK = 1 and PSZ = 3, thus total size is 4096 bytes */
#define SEEP_FINAL_BYTE_INDEX 4095

/* Define a pointer to access SmartEEPROM as bytes */
uint8_t *SmartEEPROM8 = (uint8_t *)SEEPROM_ADDR;

/* Define a pointer to access SmartEEPROM as words (32-bits) */
uint32_t *SmartEEPROM32 = (uint32_t *)SEEPROM_ADDR;

uint8_t data_8;
uint8_t ee_data_buffer[TEST_BUFF_SIZE] = {0};
	
uint8_t eeprom_menu_txt[] = "\r\n******** Enter choice ******** \r\n \
1. Reset device\r\n \
2. Back to main menu\r\n \
2. Read SmartEEPROM\r\n \
3. Write SmartEEPROM\r\n";


/**
  * To invert the data at the given index in SmartEEPROM
  */
void invert_seep_byte(uint8_t index)
{
	/* Wait till the SmartEEPROM is free */
	while (hri_nvmctrl_get_SEESTAT_BUSY_bit(NVMCTRL));

	/* Read the data, invert it, and write it back */
	data_8 = SmartEEPROM8[index];
	printf("\r\nData at test address %d is = %d\r\n", index, (int)data_8);
	SmartEEPROM8[index] = !data_8;
	printf("\r\nInverted the data at test address and written\r\n");
}


/**
  * Verify the custom data at initial 4 bytes of SmartEEPROM
  */
int8_t verify_seep_signature(void)
{
	int8_t ret_val = ERR_INVALID_DATA;

	while (hri_nvmctrl_get_SEESTAT_BUSY_bit(NVMCTRL));

	/* If SBLK fuse is not configured, inform the user and wait here */
	if (!(hri_nvmctrl_read_SEESTAT_SBLK_bf(NVMCTRL))) {
		printf("\r\nPlease configure SBLK fuse to allocate SmartEEPROM area\r\n");
		while (1);
	}

	if (SMEE_CUSTOM_SIG == SmartEEPROM32[0]) {
		ret_val = ERR_NONE;
	}

	return ret_val;
}


/**
  * Print a given array as a hex values
  */
void print_hex_array(void *mem, uint16_t len)
{
	uint16_t       i;
	unsigned char *p = (unsigned char *)mem;

	for (i = 0; i < len; i++) {
		printf("%02d ", p[i]);
		if ((i % 8 == 0) && i)
			printf("\r\n");
	}
	printf("\r\n");
}


/**
  * Initialize EEPROM usage for menu
  */
void EEPROM_init(void)
{
	if (ERR_NONE == verify_seep_signature()) {
		printf("\r\nSmartEEPROM contains valid data \r\n");
	} else {
		printf("\r\nStoring signature to SmartEEPROM address 0x00 to 0x03\r\n");
		while (hri_nvmctrl_get_SEESTAT_BUSY_bit(NVMCTRL));
		SmartEEPROM32[0] = SMEE_CUSTOM_SIG;
	}
	printf("\r\nFuse values for SBLK = %d, PSZ = %d. See the table 'SmartEEPROM Virtual \
		Size in Bytes' in the Datasheet to calculate total available bytes \r\n",
		(int)hri_nvmctrl_read_SEESTAT_SBLK_bf(NVMCTRL),
		(int)hri_nvmctrl_read_SEESTAT_PSZ_bf(NVMCTRL));

	/* Toggle a SmartEEPROM byte and give indication with LED0 on SAM E54 Xpro */
	invert_seep_byte(SEEP_TEST_ADDR);

	/* Check the data at test address and show indication on LED0 */
	if (SmartEEPROM8[SEEP_TEST_ADDR]) {
		gpio_set_pin_level(LED0, true);
	} else {
		gpio_set_pin_level(LED0, false);
	}
}


void EEPROM_menu(void)
{
	uint32_t user_selection = 0;

	while (1) {
		printf("%s", eeprom_menu_txt);
	
		if (scanf("%"PRIu32"", &user_selection) == 0) {
			/* If its not a number, flush stdin */
			fflush(stdin);
		}
	
		printf("\r\nSelected option is %d\r\n", (int)user_selection);
	
		switch (user_selection) {
			case 1:
				NVIC_SystemReset();
				return;
		
			case 2:
				printf("\r\nReturning to main menu\r\n");
				return;
		
			case 3:
				EEPROM_read();
				break;
		
			case 4:
				EEPROM_write();
				break;
		
			default:
				printf("\r\nInvalid option\r\n");
				break;
		}
	}
}


/** 
  * Code to read from EEPROM 
  */
void EEPROM_read(void) 
{
	uint8_t i = 0;
	
	for (i = 0; i < TEST_BUFF_SIZE; i++) {
		ee_data_buffer[i] = SmartEEPROM8[i];
	}
	printf("\r\nEEPROM Data at first %d locations: \r\n", TEST_BUFF_SIZE);
	print_hex_array(ee_data_buffer, TEST_BUFF_SIZE);
}


/**
  * Code to write EEPROM. Tested with SBLK = 1 and PSZ = 03
  * Thus the highest address is 4095 (See datasheet for the more details).
  */
void EEPROM_write(void)
{
	uint32_t ee_data = 0;
	uint32_t ee_addr = 0;
	uint8_t i        = 0;
	
	printf("\r\nEnter address >> ");
	scanf("%"PRIu32"", &ee_addr);
	
	if (ee_addr > SEEP_FINAL_BYTE_INDEX) {
		printf("\r\nERROR: Address invalid. Try again \r\n");
		return;
	}
	printf("\r\nEnter data >> ");
	scanf("%"PRIu32"", &ee_data);
	
	SmartEEPROM8[ee_addr] = ee_data;
	
	printf("\r\nWritten %d at %d", (int)ee_data, (int)ee_addr);
	
	for (i = 0; i < TEST_BUFF_SIZE; i++) {
		ee_data_buffer[i] = SmartEEPROM8[i];
	}
	printf("\r\nEEPROM Data at first %d locations: \r\n", TEST_BUFF_SIZE);
	print_hex_array(ee_data_buffer, TEST_BUFF_SIZE);
}
