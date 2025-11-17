#include "pdm_logic.h"

void adjust_fanpwm_callback(uint8_t data[8]) {
  uint8_t duty_255 = data[3];
  uint8_t duty_for_adjust = (uint8_t)((uint16_t)duty_255 * 200u / 255u);
  adjust_fan(duty_for_adjust);
}

void adjust_fan(uint8_t data) {
  uint16_t CCR_Width;
  uint32_t ARR_Value = (TIM3->ARR) * data;
  CCR_Width = ARR_Value / 200u;

  TIM3->CCR3 = CCR_Width;
  TIM3->CR1 |= TIM_CR1_CEN;
  TIM3->CCER |= TIM_CCER_CC3E;
}

__weak void pid_calc_error_handler(void) {
  //...
}

uint8_t pid_calc(uint8_t id) {
  if (id > 0x3F)
    pid_calc_error_handler();

  uint8_t id_buf[6];

  for (uint8_t i = 0; i < 6; i++)
    id_buf[i] = (id >> i) & 0x01;

  // parity bit calculations
  uint8_t p0 = (id_buf[0] ^ id_buf[1] ^ id_buf[2] ^ id_buf[4]) & 0x01;
  uint8_t p1 = (~(id_buf[1] ^ id_buf[3] ^ id_buf[4] ^ id_buf[5])) & 0x01;

  id |= ((p0 << 6) | (p1 << 7));
  return id;
}

uint8_t checksum_calc(uint8_t pid, uint8_t *data, uint8_t size) {
  uint8_t buf[size + 2];
  uint16_t sum = 0;
  buf[0] = pid;
  for (uint8_t i = 0; i < size; i++)
    buf[i + 1] = data[i];

  for (uint8_t i = 0; i < size + 1; i++) {
    sum += buf[i];
    if (sum > 0xFF)
      sum -= 0xFF;
  }
  sum = 0xFF - sum;
  return sum;
}
