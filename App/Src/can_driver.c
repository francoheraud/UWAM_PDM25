
#include "can_driver.h"


/**
 * CAN Driver for the 2025 PDM
 * - Franco Heraud
 */


HAL_StatusTypeDef CAN_InitDriver(CAN_Driver_t *can, CAN_HandleTypeDef *hcan) {
	if (!can)
		return HAL_ERROR;

	if (hcan->Instance == NULL)
		return HAL_ERROR;

	can->hcan = hcan;

	// still must call HAL_CAN_Init!
	HAL_StatusTypeDef can_status = HAL_CAN_Start(can->hcan);

	// ... filter config? leave for now...

	HAL_CAN_ActivateNotification(can->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	return can_status;
}


HAL_StatusTypeDef CAN_Transmit(CAN_Driver_t *can) {
	uint32_t mbox;
	can->tx.IDE  = CAN_ID_STD; // working with standard ids only
	can->tx.RTR  = CAN_RTR_DATA;
	can->tx.DLC  = can->len;
	can->tx.StdId= can->id;
	return HAL_CAN_AddTxMessage(can->hcan, &can->tx, can->tx_data, &mbox);
}


HAL_StatusTypeDef CAN_Receive(CAN_Driver_t *can) {
	can->rx.IDE = CAN_ID_STD;
	can->rx.RTR = CAN_RTR_DATA;
	can->rx.DLC = can->len;
	can->rx.StdId = can->id;

	// checks if there is a non-zero amount of can msgs stored in the rx fifo buffer
	if (HAL_CAN_GetRxFifoFillLevel(can->hcan, CAN_RX_FIFO0) != 0) // non-blocking
		return HAL_CAN_GetRxMessage(can->hcan, CAN_RX_FIFO0, &can->rx, can->rx_data);
	return HAL_BUSY;
}
