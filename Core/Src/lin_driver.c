#include "lin_driver.h"


void send_lin_msg(UART_HandleTypeDef *uart) {
	uint8_t lin_tx_data[8];
	lin_tx_data[0] = 0x55;
	lin_tx_data[1] = pid_calc(0x39);

	for (uint8_t i = 0; i < 8; i++) lin_tx_data[i + 2] = i;
	lin_tx_data[10] = checksum_calc(lin_tx_data[1], lin_tx_data + 10, 8);
	HAL_LIN_SendBreak(uart);
	HAL_UART_Transmit(uart, lin_tx_data, 11, 1000);
	HAL_Delay(1000);
}
