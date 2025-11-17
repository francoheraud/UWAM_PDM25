
#include "can_driver.h"

__weak void can_init_handler(void) {}

HAL_StatusTypeDef can_init(can_drv_t *can, CAN_HandleTypeDef *hcan) {
  if (!hcan)
    return HAL_ERROR;
  can->hcan = &hcan;
  return HAL_CAN_Start(can->hcan);
}

HAL_StatusTypeDef can_transmit(can_drv_t *can) {
  uint32_t mbox;
  can->tx.DLC = can->len;
  can->tx.StdId = can->id;
  can->tx.IDE = CAN_ID_STD;
  can->tx.RTR = CAN_RTR_DATA;
  return HAL_CAN_AddTxMessage(can->hcan, &can->tx, can->tx_data, &mbox);
}

__weak void can_error_handler(void) {}
__weak void can_receive_callback(void) {}

HAL_StatusTypeDef can_receive(can_drv_t *can) {
  if (HAL_CAN_GetRxFifoFillLevel(can->hcan, CAN_RX_FIFO0) == 0)
    return HAL_OK; // nothing to read...

  if (HAL_CAN_GetRxMessage(can->hcan, CAN_RX_FIFO0, &can->rx, can->rx_data) ==
      HAL_ERROR) {
    can_error_handler();
    return HAL_ERROR;
  }
  adjust_fanpwm_callback(can->rx_data);
  return HAL_OK;
}
