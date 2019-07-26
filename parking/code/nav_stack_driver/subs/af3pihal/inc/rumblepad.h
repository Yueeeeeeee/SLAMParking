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
#ifndef INC_RUMBLEPAD_H_
#define INC_RUMBLEPAD_H_

#include <stdint.h>

#define RUMBLEPAD_AXIS_LEFT_STICK_HORIZONTAL    0
#define RUMBLEPAD_AXIS_LEFT_STICK_VERTICAL      1
#define RUMBLEPAD_AXIS_L2                       2
#define RUMBLEPAD_AXIS_RIGHT_STICK_HORIZONTAL   3
#define RUMBLEPAD_AXIS_RIGHT_STICK_VERTICAL     4
#define RUMBLEPAD_AXIS_R2                       5
#define RUMBLEPAD_AXIS_CROSS_PAD_HORIZONTAL    16
#define RUMBLEPAD_AXIS_CROSS_PAD_VERTICAL      17

#define RUMBLEPAD_BUTTON_A                    304
#define RUMBLEPAD_BUTTON_B                    305
#define RUMBLEPAD_BUTTON_Y                    308
#define RUMBLEPAD_BUTTON_X                    307
#define RUMBLEPAD_BUTTON_L1                   310
#define RUMBLEPAD_BUTTON_R1                   311
#define RUMBLEPAD_BUTTON_SELECT               314
#define RUMBLEPAD_BUTTON_START                315
#define RUMBLEPAD_BUTTON_MODE                 316
#define RUMBLEPAD_BUTTON_L3                   317
#define RUMBLEPAD_BUTTON_R3                   318

#define RUMBLEPAD_DPAD_UP                     340
#define RUMBLEPAD_DPAD_DOWN                   341
#define RUMBLEPAD_DPAD_LEFT                   342
#define RUMBLEPAD_DPAD_RIGHT                  343

#define RUMBLEPAD_BUTTON_STATE_PRESSED          1
#define RUMBLEPAD_BUTTON_STATE_NPRESSED         0

struct rumblepad_configuration {
	const char* device_id;
	uint64_t waiting_sleep_in_micros;
	void (*axis_callback)(uint16_t axis, int16_t value);
	void (*button_callback)(uint16_t button, uint16_t old_value, uint16_t new_value);
};
typedef struct rumblepad_configuration rumblepad_configuration_t;

/** Initializes the rumblepad and starts its thread. */
void rumblepad_initialize(rumblepad_configuration_t* configuration);

/** Get the axis position for the given axis. */
int16_t rumblepad_get_axis_position(uint16_t axisID);

/** Get the button state for the given button. */
int16_t rumblepad_get_button_state(uint16_t buttonID);

/** Set the rumble state */
void rumblepad_set_rumble(uint16_t strong_magnitude, uint16_t weak_magnitude, uint16_t length, uint16_t delay);

/** Terminates the rumblepad thread and frees its resources. */
void rumblepad_terminate();

#endif /* INC_RUMBLEPAD_H_ */
