#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lora.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* Private definitions and enumerations */
#define MASTERTX_SLAVERX_MSG_SIZE	2
#define NUM_OF_SLAVES				32

typedef enum {
	TDMA_ST_MASTERTX_SLAVERX = 0,
	TDMA_ST_MASTERRX_SLAVETX,
	TDMA_ST_CHECK_NEXT_SLAVE,
	TDMA_ST_EXEC_USER_PROGRAM,
} tdma_state_t;

typedef struct {
	bool erase_address_cmd;
	bool available_slave;
	bool inputs[4];
	bool outputs[4];
} slave_ctrl_t;

/* Private variables */
uint8_t mastertx_msg[MASTERTX_SLAVERX_MSG_SIZE];
uint8_t masterrx_msg;

tdma_state_t tdma_state = TDMA_ST_MASTERTX_SLAVERX;
slave_ctrl_t slave_ctrl[NUM_OF_SLAVES] = {0};
bool new_message_arrived = false;

void lora_rx_done_callback(uint8_t* buffer_rx, int pac_size)
{
	memset(&masterrx_msg, 0, sizeof(masterrx_msg));
	memcpy(&masterrx_msg, buffer_rx, sizeof(masterrx_msg));
	new_message_arrived = true;
}

uint8_t get_next_available_slave (uint8_t current_slave_address)
{
	uint8_t next_available_slave_address = current_slave_address + 1;

	/* Check which is the next available slave */
	while( slave_ctrl[next_available_slave_address].available_slave == false ) {
		next_available_slave_address++;
		if (next_available_slave_address >= NUM_OF_SLAVES)
			next_available_slave_address = 0;
	}

	return next_available_slave_address;
}

bool is_erase_slave_address_needed (uint8_t slave_address)
{
	return slave_ctrl[slave_address].erase_address_cmd;
}

void parse_io_cmd_message (uint8_t slave_address, uint8_t* message)
{
	uint16_t raw_message = 0;

	uint16_t start_bit = 0;
	uint16_t command_bit = 0;
	uint16_t address_bits = ((uint16_t)slave_address) & 0x001F;
	uint16_t data_bits = 0;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[0])) & 0x0001) << 1;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[1])) & 0x0001) << 2;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[2])) & 0x0001) << 3;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[3])) & 0x0001) << 4;

	raw_message |= start_bit << 0;
	raw_message |= command_bit << 1;
	raw_message |= address_bits << 2;
	raw_message |= data_bits << 7;

	uint8_t number_of_ones = 0;
	for (int i = 0; i < 12; i++) {
		number_of_ones += (raw_message >> i) & 0x0001;
	}
	uint16_t parity_bit = number_of_ones % 2; // 0 if even, 1 if odd
	uint16_t end_bit = 1;
	raw_message |= parity_bit << 12;
	raw_message |= end_bit << 13;

	uint8_t lsb_message = (uint8_t)(raw_message & 0x00FF);
	uint8_t msb_message = (uint8_t)((raw_message >> 8) & 0x00FF);
	*message = lsb_message;
	message++;
	*message = msb_message;
}

void parse_erase_address_cmd_message (uint8_t slave_address, uint8_t* message)
{
	uint16_t raw_message = 0;

	uint16_t start_bit = 0;
	uint16_t command_bit = 1;
	uint16_t address_bits = ((uint16_t)slave_address) & 0x001F;
	uint16_t data_bits = 0;

	raw_message |= start_bit << 0;
	raw_message |= command_bit << 1;
	raw_message |= address_bits << 2;
	raw_message |= data_bits << 7;

	uint8_t number_of_ones = 0;
	for (int i = 0; i < 12; i++) {
		number_of_ones += (raw_message >> i) & 0x0001;
	}
	uint16_t parity_bit = number_of_ones % 2; // 0 if even, 1 if odd
	uint16_t end_bit = 1;
	raw_message |= parity_bit << 12;
	raw_message |= end_bit << 13;

	uint8_t lsb_message = (uint8_t)(raw_message & 0x00FF);
	uint8_t msb_message = (uint8_t)((raw_message >> 8) & 0x00FF);
	*message = lsb_message;
	message++;
	*message = msb_message;
}

void parse_set_new_address_cmd_message (uint8_t slave_address, uint8_t* message)
{
	uint16_t raw_message = 0;

	uint16_t start_bit = 0;
	uint16_t command_bit = 0;
	uint16_t address_bits = 0;
	uint16_t data_bits = ((uint16_t)slave_address) & 0x001F;

	raw_message |= start_bit << 0;
	raw_message |= command_bit << 1;
	raw_message |= address_bits << 2;
	raw_message |= data_bits << 7;

	uint8_t number_of_ones = 0;
	for (int i = 0; i < 12; i++) {
		number_of_ones += (raw_message >> i) & 0x0001;
	}
	uint16_t parity_bit = number_of_ones % 2; // 0 if even, 1 if odd
	uint16_t end_bit = 1;
	raw_message |= parity_bit << 12;
	raw_message |= end_bit << 13;

	uint8_t lsb_message = (uint8_t)(raw_message & 0x00FF);
	uint8_t msb_message = (uint8_t)((raw_message >> 8) & 0x00FF);
	*message = lsb_message;
	message++;
	*message = msb_message;
}

void parse_received_message_from_slave (uint8_t slave_address, uint8_t message)
{
	uint8_t number_of_ones = 0;
	for (int i = 0; i < 5; i++) {
		number_of_ones += (message >> i) & 0x01;
	}
	uint8_t parity_bit_calculated = number_of_ones % 2;
	uint8_t parity_bit_from_msg = (message >> 5) & 0x01;
	if (parity_bit_from_msg == parity_bit_calculated) {
		slave_ctrl[slave_address].inputs[0] = (bool)((message >> 1) & 0x01);
		slave_ctrl[slave_address].inputs[1] = (bool)((message >> 2) & 0x01);
		slave_ctrl[slave_address].inputs[2] = (bool)((message >> 3) & 0x01);
		slave_ctrl[slave_address].inputs[3] = (bool)((message >> 4) & 0x01);
	}
}

void execute_user_program ()
{
	//TODO Implement user program logic
}

void tdma(void *p)
{
	uint32_t tick_timeout = 0;
	uint8_t current_slave_adress = 0;

	for(;;) {
		switch (tdma_state) {
			case TDMA_ST_MASTERTX_SLAVERX:
				memset(mastertx_msg, 0, sizeof(mastertx_msg));

				/* Prepare the message to be transmitted to the slave N */
				if (is_erase_slave_address_needed(current_slave_adress)) {
					parse_erase_address_cmd_message(current_slave_adress, mastertx_msg);
				}
				else {
					parse_io_cmd_message(current_slave_adress, mastertx_msg);
				}

				lora_send_packet(mastertx_msg, sizeof(mastertx_msg));

				lora_receive();
				tick_timeout = xTaskGetTickCount();
				tdma_state = TDMA_ST_MASTERRX_SLAVETX;
				break;

			case TDMA_ST_MASTERRX_SLAVETX:
				if (tick_timeout - xTaskGetTickCount() >= pdMS_TO_TICKS(100)) {
					/* Time to listen slave N transmission has expired
					 * Pass to the next slave available on the network */
					tdma_state = TDMA_ST_CHECK_NEXT_SLAVE;
				}
				else {
					/* Verify if the master has received a message
					 * If true, update the inputs table and then
					 * pass to the next slave available on the network */
					if (new_message_arrived) {
						new_message_arrived = false;
						parse_received_message_from_slave(current_slave_adress, masterrx_msg);
						tdma_state = TDMA_ST_CHECK_NEXT_SLAVE;
					}
				}
				break;

			case TDMA_ST_CHECK_NEXT_SLAVE:
				if (get_next_available_slave(current_slave_adress) <= current_slave_adress) {
					tdma_state = TDMA_ST_EXEC_USER_PROGRAM;
				}
				else {
					tdma_state = TDMA_ST_MASTERTX_SLAVERX;
				}
				current_slave_adress = get_next_available_slave(current_slave_adress);
				break;

			case TDMA_ST_EXEC_USER_PROGRAM:
				execute_user_program();
				tdma_state = TDMA_ST_MASTERTX_SLAVERX;
				break;

			default:
				break;
		}
		vTaskDelay(1);
	}
}

void app_main()
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	lora_init();
	lora_set_frequency(915e6);
	lora_set_bandwidth(500e3);
	lora_set_spreading_factor(7);
	lora_enable_crc();
	lora_onReceive(&lora_rx_done_callback);

	xTaskCreatePinnedToCore(&tdma, "tdma", 8192, NULL, 5, NULL, 1);
}
