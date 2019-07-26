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
#ifndef INC_GAMEPAD_H_
#define INC_GAMEPAD_H_

#include <stdint.h>

#define GAMEPAD_AXIS_LEFT_STICK_HORIZONTAL	 0
#define GAMEPAD_AXIS_LEFT_STICK_VERTICAL	 1
#define GAMEPAD_AXIS_RIGHT_STICK_HORIZONTAL	 2
#define GAMEPAD_AXIS_RIGHT_STICK_VERTICAL	 3
#define GAMEPAD_AXIS_CROSS_PAD_HORIZONTAL	 4
#define GAMEPAD_AXIS_CROSS_PAD_VERTICAL		 5

#define GAMEPAD_BUTTON_1					 6
#define GAMEPAD_BUTTON_2					 7
#define GAMEPAD_BUTTON_3					 8
#define GAMEPAD_BUTTON_4					 9
#define GAMEPAD_BUTTON_L1					10
#define GAMEPAD_BUTTON_R1					11
#define GAMEPAD_BUTTON_L2					12
#define GAMEPAD_BUTTON_R2					13
#define GAMEPAD_BUTTON_SELECT				14
#define GAMEPAD_BUTTON_START				15
#define GAMEPAD_BUTTON_LEFT_STICK			16
#define GAMEPAD_BUTTON_RIGHT_STICK			17

struct gamepad_configuration {
	const char* device_id;
	uint64_t waiting_sleep_in_micros;
	void (*axis_callback)(uint8_t axis, uint16_t value);
	void (*button_callback)(uint8_t button, uint16_t old_value, uint16_t new_value);
};
typedef struct gamepad_configuration gamepad_configuration_t;

/** Initializes the gamepad and starts its thread. */
void gamepad_initialize(gamepad_configuration_t* configuration);

/** Get the axis position for the given axis. */
int16_t gamepad_get_axis_position(uint8_t axisID);

/** Get the button state for the given button. */
int16_t gamepad_get_button_state(uint8_t buttonID);

/** Terminates the gamepad thread and frees its resources. */
void gamepad_terminate();

#endif /* INC_GAMEPAD_H_ */
