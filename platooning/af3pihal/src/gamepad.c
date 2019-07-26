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
#include "gamepad.h"

#include <bits/time.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

#include "debugprint.h"
#include "timeutil.h"

#define GAMEPAD_EVENT_BUTTON 0x01
#define GAMEPAD_EVENT_AXIS   0x02
#define GAMEPAD_EVENT_INIT   0x80

union gamepad_input_events {
    struct gamepad_input {
        int16_t left_stick_horizontal;
        int16_t left_stick_vertical;
        int16_t right_stick_horizontal;
        int16_t right_stick_vertical;
        int16_t cross_pad_horizontal;
        int16_t cross_pad_vertical;
        int16_t button_1;
        int16_t button_2;
        int16_t button_3;
        int16_t button_4;
        int16_t button_left_1;
        int16_t button_right_1;
        int16_t button_left_2;
        int16_t button_right_2;
        int16_t button_select;
        int16_t button_start;
        int16_t button_left_stick;
        int16_t button_right_stick;
    } inputs;
    int16_t raw_inputs[18];
};

struct gamepad_event {
    uint32_t time;  // event time stamp in millisecons
    int16_t value;  // value
    uint8_t type;   // event type
    uint8_t number; // axis/button number
};
typedef struct gamepad_event gamepad_event_t;

static gamepad_configuration_t* configuration;

static volatile union gamepad_input_events input_events;

static pthread_t* gamepad_thread;
static bool gamepad_thread_abort;

static void* gamepad_thread_worker() {
	int device_fd = open(configuration->device_id, O_RDONLY);
	if(device_fd <= 0) {
		perror("Failed to open gamepad device.");
		return NULL;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Opened gamepad device %s.\n", configuration->device_id);

	struct timeval sleep_time;
	gamepad_event_t gp_event;
	fd_set reads;


	while(!gamepad_thread_abort) {
		FD_ZERO(&reads);
		FD_SET(device_fd, &reads);

		sleep_time.tv_sec = 0;
		sleep_time.tv_usec = configuration->waiting_sleep_in_micros;
		int result = select(device_fd + 1, &reads, NULL, NULL, &sleep_time);
		if(result > 0 && FD_ISSET(device_fd, &reads)) {
			if(read(device_fd, &gp_event, sizeof(gp_event)) == sizeof(gp_event)) {
				gp_event.type &= ~GAMEPAD_EVENT_INIT;
				if(gp_event.type == GAMEPAD_EVENT_AXIS) {
					input_events.raw_inputs[gp_event.number] = gp_event.value;
					if(configuration->axis_callback != NULL) {
						configuration->axis_callback(gp_event.number, gp_event.value);
					}
				}
				if(gp_event.type == GAMEPAD_EVENT_BUTTON) {
					uint8_t button = gp_event.number + 6;
					uint16_t old_value = input_events.raw_inputs[button];
					input_events.raw_inputs[button] = gp_event.value;
					if(configuration->button_callback != NULL) {
						configuration->button_callback(button, old_value, gp_event.value);
					}
				}
			}
		}
	}
	close(device_fd);
	return NULL;
}

void gamepad_initialize(struct gamepad_configuration* config) {
	if(config == NULL) {
		perror("Gamepad configuration is NULL.");
		return;
	}
	configuration = config;
	gamepad_thread = malloc(sizeof(pthread_t));
	if(gamepad_thread == NULL) {
		perror("Out of memory during malloc().");
		return;
	}
	gamepad_thread_abort = false;
	pthread_create(gamepad_thread, NULL, gamepad_thread_worker, NULL);
}

int16_t gamepad_get_axis_position(uint8_t axisID) {
	if(axisID >= GAMEPAD_AXIS_LEFT_STICK_HORIZONTAL && axisID <= GAMEPAD_AXIS_CROSS_PAD_VERTICAL) {
		return input_events.raw_inputs[axisID];
	}
	return 0;
}

int16_t gamepad_get_button_state(uint8_t buttonID) {
	if(buttonID >= GAMEPAD_BUTTON_1 && buttonID <= GAMEPAD_BUTTON_RIGHT_STICK) {
		return input_events.raw_inputs[buttonID];
	}
	return 0;
}


void gamepad_terminate() {
	gamepad_thread_abort = true;
	if(gamepad_thread != NULL) {
		pthread_join(*gamepad_thread, NULL);
		debug_print(DEBUG_PRINT_LEVEL_FEW, "Gamepad thread terminated.\n");
	}
}

