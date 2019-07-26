#include "temp_actuator.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include "debugprint.h"

#define UPPER_7BITS(x)	(uint8_t)((x >> 7) & 0x7F)
#define LOWER_7BITS(x)	(uint8_t)(x & 0x7F)
#define GET_COMMAND		0x90
#define SET_COMMAND		0x84

static int device_fd;

int temp_actuator_initialize(const char * device) {
	device_fd = open(device, O_RDWR | O_NOCTTY);
	if(device_fd <= 0) {
		printf("Error opening actuator device. %s\n", device);
		return -1;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Opened actuator device %s.\n", device);

	struct termios options;
	tcgetattr(device_fd, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~(ONLCR | OCRNL);
	tcsetattr(device_fd, TCSANOW, &options);
	return device_fd;
}


int temp_actuator_get_position(uint8_t channel) {
    uint8_t command[] = {GET_COMMAND, channel};
    if(write(device_fd, command, sizeof(command)) == -1)
    {
        perror("Error writing actuator command.");
        return -1;
    }

    unsigned char response[2];
    if(read(device_fd,response,2) != 2)
    {
        perror("Error reading actuator response.");
        return -1;
    }
    return response[0] + (response[1] << 8);
}

int temp_actuator_set_target(uint8_t channel, uint16_t target) {
    uint8_t command[] = {SET_COMMAND, channel, LOWER_7BITS(target), UPPER_7BITS(target)};
    if (write(device_fd, command, sizeof(command)) == -1)
    {
        perror("Error writing actuator command.");
        return -1;
    }
    return 0;
}

int temp_actuator_device_set_target(int device, uint8_t channel, uint16_t target) {
	if(device <= 0) {
		return -1;
	}
	uint8_t command[] = {SET_COMMAND, channel, LOWER_7BITS(target), UPPER_7BITS(target)};
	if (write(device, command, sizeof(command)) == -1) {
		return -1;
	}
	return 0;
}


void temp_actuator_terminate() {
	if(device_fd > 0) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "Temporary LED actuator terminated.\n");
		close(device_fd);
	}
}
