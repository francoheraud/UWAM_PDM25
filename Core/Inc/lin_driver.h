
#ifndef INC_LIN_DRIVER_H_
#define INC_LIN_DRIVER_H_

// very bare bones:

#include "main.h"
#include "pdm_logic.h"
#include <stdint.h>


void send_lin_msg(UART_HandleTypeDef *uart);


#endif /* INC_LIN_DRIVER_H_ */
