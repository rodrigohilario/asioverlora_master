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

#define MASTERTX_SLAVERX_MSG_SIZE 2
#define MASTERRX_SLAVETX_MSG_SIZE 1

typedef enum {
    TDMA_ST_MASTERTX_SLAVERX = 0,
    TDMA_ST_MASTERRX_SLAVETX,
	TDMA_ST_CHECK_NEXT_SLAVE,
} tdma_state_t;

 uint8_t mastertx_msg[MASTERTX_SLAVERX_MSG_SIZE];
 uint8_t masterrx_msg[MASTERRX_SLAVETX_MSG_SIZE];

tdma_state_t tdma_state = TDMA_ST_MASTERTX_SLAVERX;
bool available_slave[32] = {0};
bool new_message_arrived = false;

void lora_rx_done_callback(uint8_t* buffer_rx, int pac_size)
{
	memset(masterrx_msg, 0, sizeof(masterrx_msg));
	memcpy(masterrx_msg, buffer_rx, sizeof(masterrx_msg));
    new_message_arrived = true;
}

uint8_t get_next_available_slave (uint8_t current_slave)
{
	uint8_t next_available_slave = current_slave + 1;

	/* Check which is the next available slave */
	while( available_slave[next_available_slave] == false ) {
		next_available_slave++;
		if (next_available_slave >= 32)
			next_available_slave = 0;
	}

	return next_available_slave;
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
				/* ... */
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
					 * If true, pass to the next slave available on the network */
					if (new_message_arrived) {
						new_message_arrived = false;
						tdma_state = TDMA_ST_CHECK_NEXT_SLAVE;
					}
				}
				break;

			case TDMA_ST_CHECK_NEXT_SLAVE:
				current_slave_adress = get_next_available_slave(current_slave_adress);
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
