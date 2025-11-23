/*
 * pdm_logic.h
 *
 *  Created on: Nov 18, 2025
 *      Author: Franc
 */

#ifndef INC_PDM_LOGIC_H_
#define INC_PDM_LOGIC_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lin_driver.h"
#include "can_driver.h"

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart4;
extern CAN_HandleTypeDef hcan;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

#define NUM_CHANNELS		7
#define NUM_FUSES			7		// one fuse per switch circuit
#define PDM_PWM_RES			255
#define NUM_CURRENT_INPUTS	7

GPIO_TypeDef *fuse_input_ports[NUM_FUSES] = {
		GPIOC,
		GPIOC,
		GPIOA,
		GPIOB,
		GPIOA,
		GPIOC,
		GPIOB
};

const uint16_t fuse_inputs[NUM_FUSES] = {
		1,
		3,
		3,
		7,
		11,
		5,
		10
};

typedef struct {
	uint32_t code[NUM_CURRENT_INPUTS];
	float volts[NUM_CURRENT_INPUTS];
	float amps[NUM_CURRENT_INPUTS];
} ADC_t;

typedef struct {
	uint8_t fuse;
	bool mcu;
} Status_t;

typedef struct {
	CAN_Driver_t can;
	ADC_t adc;
	Status_t state;
} PDM_t;

#endif /* INC_PDM_LOGIC_H_ */
