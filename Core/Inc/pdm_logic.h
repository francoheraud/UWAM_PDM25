#include "main.h"
#include <stdint.h>
void adjust_fanpwm_callback(uint8_t data[8]);
void adjust_fan(uint8_t data);
uint8_t pid_calc(uint8_t id);
uint8_t checksum_calc(uint8_t pid, uint8_t *data, uint8_t size);
