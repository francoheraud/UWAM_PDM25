
#include "main.h"
#include "pdm_logic.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  CAN_HandleTypeDef *hcan;
  CAN_TxHeaderTypeDef tx;
  CAN_RxHeaderTypeDef rx;
  uint8_t len, tx_data[8], rx_data[8];
  uint32_t id;
} can_drv_t;

HAL_StatusTypeDef can_init(can_drv_t *can, CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef can_transmit(can_drv_t *can);
HAL_StatusTypeDef can_receive(can_drv_t *can);
