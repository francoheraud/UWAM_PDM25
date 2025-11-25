/**
 * Main PDM code:
 *
 * =>
 * => Current Sensing
 */

#include "pdm_logic.h"

HAL_StatusTypeDef can_state = HAL_OK;
uint32_t adc_ch2[6], adc_ch3;

/**
 * @brief Helper constrain function.
 * @param float x, float low, float high
 * @return x -> Constrained between low and high.
 */
static float Constrain(float x, float low, float high) {
  if (x < low)
    return low;
  if (x > high)
    return high;
  return x;
}

/**
 * @brief Main PDM init function.
 * @param pdm
 */
HAL_StatusTypeDef PDM_Init(PDM_t *pdm) {

  // init can...
  CAN_InitDriver(&pdm->can, &hcan);

  // init adc...
  if (HAL_ADC_Start_DMA(&hadc2, adc_ch2, 6) != HAL_OK)
    return HAL_ERROR;

  if (HAL_ADC_Start(&hadc3) != HAL_OK)
    return HAL_ERROR;

  adc_ch3 = HAL_ADC_GetValue(&hadc3);

  // init struct variables...
  pdm->state.fuse = 0;
  pdm->state.mcu = false;

  return HAL_OK;
}

void Poll_ADC_Conversions(PDM_t *pdm) {
  // ..
}

/**
 * @brief Sets the PWM duty cycle for PWM_FANS_PWR.
 * @param data Input adc command from VCU (to be extracted from CAN rx buffer).
 * @note Simplified previous baremetal implementation by using a macro provided
 * by HAL.
 */
void PWM_Set_Fans(uint8_t *can_rx_data) {
  uint8_t duty_adjusted = (uint16_t)(can_rx_data[3]) * 200u / 255u;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_adjusted);
}

/**
 * @brief Reads current sense amplifier inputs and converts them into switch
 * circuit currents for each channel.
 * @param pdm
 */
void Store_Current_Readings(PDM_t *pdm) {
  const float v_supply = 3.3f; // V
  const float gain = 50.0f;
  const float r_shunt = 0.002; // ohms

  for (uint8_t i = 0; i < NUM_CURRENT_INPUTS; i++) {
    pdm->adc.volts[i] = 3.3f * (float)(pdm->adc.code / ADC_RES);
    pdm->adc.volts[i] = Constrain(pdm->adc.volts[i], 0.0f, v_supply);
    pdm->adc.amps[i] = pdm->adc.volts[i] / (gain * r_shunt);
  }
}

/**
 * @brief Detects whether a fuse in each channel has blown.
 * @param pdm
 * @note Pull up resistor + n-channel mosfet means:
 * 0: Normal operation
 * 1: Fuse has blown due to overcurrent
 */
void Store_Fuse_Status(PDM_t *pdm) {
  for (uint8_t i = 0; i < NUM_FUSES; i++) {
    GPIO_PinState state = HAL_GPIO_ReadPin(fuse_input_ports[i], fuse_inputs[i]);
    pdm->state.fuse |= ((uint8_t)state & 0xff) << i;
  }
}

static void Clear_Tx_Buffer(PDM_t *pdm) {
  memset((void *)pdm->can.tx_data, 0, sizeof(pdm->can.tx_data));
}

/**
 *
 * @param pdm
 */
void Transmit_All_Data(PDM_t *pdm) {
  Store_Fuse_Status(pdm);
  Store_Current_Readings(pdm);

  // Current inputs
  Clear_Tx_Buffer(pdm);
  pdm->can.len = 7;
  pdm->can.id = PDM_CAN_ID;
  uint8_t amps_u[NUM_CURRENT_INPUTS] = {0};
  for (uint8_t i = 0; i < NUM_CURRENT_INPUTS; i++) {
    amps_u[i] = (uint8_t)Constrain(pdm->adc.amps[i], 0.0f, 255.0f);
    pdm->can.tx_data[i] = amps_u[i];
  }
  can_state = CAN_Transmit(&pdm->can);

  // Status bits
  Clear_Tx_Buffer(pdm);
  pdm->can.len = 1;
  pdm->can.id = PDM_CAN_ID + 1;
  pdm->can.tx_data[0] = pdm->state.fuse;
  pdm->can.tx_data[1] = pdm->state.mcu;
  can_state = CAN_Transmit(&pdm->can);

  LIN_Transmit(&huart4);
}
