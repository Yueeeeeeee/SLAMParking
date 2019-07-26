#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdint.h>
#include "datatypes.h"

uint8_t get_fw();

uint8_t send_alive();
uint8_t reboot_device();

uint8_t get_values(mc_values*);

void set_velocity(double);
double get_velocity();

uint8_t set_rpm(int rpm);
double get_rpm();

#endif /* COMMANDS_H_ */
