/*
 * can_driver.h
 *
 *  Created on: Nov 18, 2025
 *      Author: Franc
 */

#ifndef SRC_CAN_DRIVER_H_
#define SRC_CAN_DRIVER_H_

#include "main.h"
#include <stdint.h>


#define PDM_CAN_ID	0x030

typedef struct {
	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef tx;
	CAN_RxHeaderTypeDef rx;
	uint8_t tx_data[8], rx_data[8];
	uint32_t id;
	uint8_t len;
} CAN_Driver_t;

HAL_StatusTypeDef CAN_InitDriver(CAN_Driver_t *can, CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Transmit	(CAN_Driver_t *can);
HAL_StatusTypeDef CAN_Receive	(CAN_Driver_t *can);


#endif /* SRC_CAN_DRIVER_H_ */
