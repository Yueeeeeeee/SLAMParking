/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Thomas Boehm, Florian Hoelzl - initial API and implementation
 *******************************************************************************/
#include "rumblepad.h"

#include <bits/time.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "debugprint.h"
#include "timeutil.h"

#define RUMBLEPAD_EVENT_BUTTON 0x01
#define RUMBLEPAD_EVENT_AXIS   0x03
#define RUMBLEPAD_EVENT_INIT   0x80


const int AXIS_MAPPINGS[] =  {0, 1, 2, 3, 4, 5, 16, 17};
const int AXIS_MAPPINGS_SIZE  = 8;
const int BUTTON_MAPPINGS[] =  {304, 305, 307, 308, 310, 311, 314, 315, 316, 317, 318};
const int BUTTON_MAPPINGS_SIZE  = 11;

union rumblepad_input_events {
    struct rumblepad_input {
        int16_t left_stick_horizontal;
        int16_t left_stick_vertical;
        int16_t axis_l2;
        int16_t right_stick_horizontal;
        int16_t right_stick_vertical;
        int16_t axis_r2;
        int16_t cross_pad_horizontal;
        int16_t cross_pad_vertical;
        int16_t button_a;
        int16_t button_b;
        int16_t button_y;
        int16_t button_x;
        int16_t button_l1;
        int16_t button_r1;
        int16_t button_select;
        int16_t button_start;
        int16_t button_mode;
        int16_t button_l3;
        int16_t button_r3;
    } inputs;
    int16_t raw_inputs[18];
};

struct rumblepad_event {
    uint64_t time;  // event time stamp in millisecons
    uint16_t type;
    uint16_t code;
    int16_t value;  // value
    uint16_t sync; // axis/button number
};
typedef struct rumblepad_event rumblepad_event_t;

static rumblepad_configuration_t* configuration;

static volatile union rumblepad_input_events input_events;

static pthread_t* rumblepad_thread;
static bool rumblepad_thread_abort;
static struct ff_effect rumble_effect;
static bool rumble_changed;

static void* rumblepad_thread_worker() {
	struct timeval sleep_time;
	rumblepad_event_t gp_event;
	fd_set reads;

	int device_fd = open(configuration->device_id, O_RDWR);
	if(device_fd <= 0) {
		perror("Failed to open rumblepad device.");
		return NULL;
	}

	while(!rumblepad_thread_abort) {
		FD_ZERO(&reads);
		FD_SET(device_fd, &reads);

		sleep_time.tv_sec = 0;
		sleep_time.tv_usec = configuration->waiting_sleep_in_micros;
		int result = select(device_fd + 1, &reads, NULL, NULL, &sleep_time);
		if(result > 0 && FD_ISSET(device_fd, &reads)) {
			if(read(device_fd, &gp_event, sizeof(gp_event)) == sizeof(gp_event)) {
				if(gp_event.type == RUMBLEPAD_EVENT_AXIS) {
					for(int i = 0; i < AXIS_MAPPINGS_SIZE; i++) {
						if (gp_event.code == AXIS_MAPPINGS[i]) {
							input_events.raw_inputs[i] = gp_event.value;
							if(configuration->axis_callback != NULL) {
								configuration->axis_callback(gp_event.code, gp_event.value);
							}
							break;
						}
					}
				}
				if(gp_event.type == RUMBLEPAD_EVENT_BUTTON) {
					for(int i = 0; i < BUTTON_MAPPINGS_SIZE; i++) {
						if (gp_event.code == BUTTON_MAPPINGS[i]) {
							uint16_t old_value = input_events.raw_inputs[i + AXIS_MAPPINGS_SIZE];
							input_events.raw_inputs[i + AXIS_MAPPINGS_SIZE] = gp_event.value;
							if(configuration->button_callback != NULL) {
								configuration->button_callback(gp_event.code, old_value , gp_event.value);
							}
							break;
						}
					}
				}
			}
		}

		if(rumble_changed) {
			if(ioctl(device_fd, EVIOCSFF, &rumble_effect) != -1) {
				struct input_event play;
				play.type = EV_FF;
				play.code = rumble_effect.id;
				printf("id: %i\n", rumble_effect.id);
				play.value = 1;
				write(device_fd, (const void*) &play, sizeof(play));
			}
			else
			{
				int err = errno;
				printf("Error: Writing input event!\n");
				switch(err) {
				case EBADF:
					printf("Not a valid file descriptor.\n");
					break;
				case EFAULT:
					printf("Struct inaccessible.\n");
					break;
				case EINVAL:
					printf("Request invalid\n");
					break;
				case ENOTTY:
					printf("fd not associated with character special device\n");
					break;
				default:
					printf("Errno: %i\n", err);
					break;
				}
			}
			rumble_changed = false;
		}

	}
	close(device_fd);
	return NULL;
}

void rumblepad_initialize(struct rumblepad_configuration* config) {
	if(config == NULL) {
		perror("Gamepad configuration is NULL.");
		return;
	}
	configuration = config;
	rumblepad_thread = malloc(sizeof(pthread_t));
	if(rumblepad_thread == NULL) {
		perror("Out of memory during malloc().");
		return;
	}
	rumblepad_thread_abort = false;
	rumble_changed = false;

	rumble_effect.type = FF_RUMBLE;
	rumble_effect.id = -1;

	pthread_create(rumblepad_thread, NULL, rumblepad_thread_worker, NULL);
}

int16_t rumblepad_get_axis_position(uint16_t axisID) {
	for(int i = 0; i < AXIS_MAPPINGS_SIZE; i++) {
		if (axisID == AXIS_MAPPINGS[i]) {
			if(axisID == RUMBLEPAD_AXIS_L2 || axisID == RUMBLEPAD_AXIS_R2)
				return input_events.raw_inputs[i];
			else
				return input_events.raw_inputs[i] - (INT16_MAX) - 1;
		}
	}
	return 0;
}

int16_t rumblepad_get_button_state(uint16_t buttonID) {
	if(buttonID >= 340) {
		//CROSS_PAD
		switch(buttonID) {
		case RUMBLEPAD_DPAD_UP:
			for(int i = 0; i < AXIS_MAPPINGS_SIZE; i++) {
				if(AXIS_MAPPINGS[i] == RUMBLEPAD_AXIS_CROSS_PAD_VERTICAL) {
					return input_events.raw_inputs[i] == -1;
				}
			}
			break;
		case RUMBLEPAD_DPAD_DOWN:
			for(int i = 0; i < AXIS_MAPPINGS_SIZE; i++) {
				if(AXIS_MAPPINGS[i] == RUMBLEPAD_AXIS_CROSS_PAD_VERTICAL) {
					return input_events.raw_inputs[i] == 1;
				}
			}
			break;
		case RUMBLEPAD_DPAD_LEFT:
			for(int i = 0; i < AXIS_MAPPINGS_SIZE; i++) {
				if(AXIS_MAPPINGS[i] == RUMBLEPAD_AXIS_CROSS_PAD_HORIZONTAL) {
					return input_events.raw_inputs[i] == -1;
				}
			}
			break;
		case RUMBLEPAD_DPAD_RIGHT:
			for(int i = 0; i < AXIS_MAPPINGS_SIZE; i++) {
				if(AXIS_MAPPINGS[i] == RUMBLEPAD_AXIS_CROSS_PAD_HORIZONTAL) {
					return input_events.raw_inputs[i] == 1;
				}
			}
			break;
		}
	}
	else {
		for(int i = 0; i < BUTTON_MAPPINGS_SIZE; i++) {
			if(buttonID == BUTTON_MAPPINGS[i])
				return input_events.raw_inputs[i + AXIS_MAPPINGS_SIZE];
		}
	}
	return 0;
}

void rumblepad_set_rumble(uint16_t strong_magnitude, uint16_t weak_magnitude, uint16_t length, uint16_t delay) {
	rumble_effect.u.rumble.strong_magnitude = strong_magnitude;
	rumble_effect.u.rumble.weak_magnitude = weak_magnitude;
	rumble_effect.replay.length = length;
	rumble_effect.replay.delay = delay;
	rumble_changed = true;
}


void rumblepad_terminate() {
	rumblepad_thread_abort = true;
	if(rumblepad_thread != NULL) {
		pthread_join(*rumblepad_thread, NULL);
		debug_print(DEBUG_PRINT_LEVEL_FEW, "Gamepad thread terminated.\n");
	}
}

