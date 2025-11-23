#include "lin_driver.h"


__weak void Pid_Calc_Error_Handler() {
	// ...
	return;
}


static uint8_t Pid_Calc(uint8_t id) {
	if (id > 0x3F)
	    Pid_Calc_Error_Handler();


	uint8_t id_buf[6];

	for (uint8_t i = 0; i < 6; i++)
		id_buf[i] = (id >> i) & 0x01;

	// parity bit calculations
	uint8_t p0 = (id_buf[0] ^ id_buf[1] ^ id_buf[2] ^ id_buf[4]) & 0x01;
	uint8_t p1 = (~(id_buf[1] ^ id_buf[3] ^ id_buf[4] ^ id_buf[5])) & 0x01;

	id |= ((p0 << 6) | (p1 << 7));
	return id;
}

static uint8_t Checksum_Calc(uint8_t pid, uint8_t *data, uint8_t size) {

	uint8_t buf[size + 2];
	uint16_t sum = 0;
	buf[0] = pid;
	for (uint8_t i = 0; i < size; i++)
		buf[i + 1] = data[i];

	for (uint8_t i = 0; i < size + 1; i++) {
		sum += buf[i];
	    if (sum > 0xFF) sum -= 0xFF;
	  }

	sum = 0xFF - sum;
	return sum;
}


void LIN_Transmit(UART_HandleTypeDef *uart) {

	uint8_t lin_tx_data[20];

	lin_tx_data[0] = 0x55;
	lin_tx_data[1] = Pid_Calc(0x39);

	for (uint8_t i = 0; i < 8; i++) lin_tx_data[i + 2] = i;
	lin_tx_data[10] = Checksum_Calc(lin_tx_data[1], lin_tx_data + 10, 8);

	HAL_LIN_SendBreak(uart);
	HAL_UART_Transmit(uart, lin_tx_data, 11, 1000);
	HAL_Delay(1000);
}
