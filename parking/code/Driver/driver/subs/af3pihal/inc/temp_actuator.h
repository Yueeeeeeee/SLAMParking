#ifndef INC_TEMP_ACTUATOR_H_
#define INC_TEMP_ACTUATOR_H_

#include <stdint.h>

int temp_actuator_initialize(const char * device);

int temp_actuator_get_position(uint8_t channel);

int temp_actuator_set_target(uint8_t channel, uint16_t target);

int temp_actuator_device_set_target(int device, uint8_t channel, uint16_t target);

void temp_actuator_terminate();

#endif /* INC_TEMP_ACTUATOR_H_ */
