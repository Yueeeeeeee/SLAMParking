/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *    Constantin Dresel - added recceive function
 *******************************************************************************/

#ifndef INC_PROTOCOL_CONTROL_CENTER_H_
#define INC_PROTOCOL_CONTROL_CENTER_H_

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

struct protocol_control_center_configuration {
	char* server_ip_address;
	uint16_t server_port;
};
typedef struct protocol_control_center_configuration protocol_control_center_configuration_t;

/** Create the connection to the control center. */
void protocol_control_center_create(protocol_control_center_configuration_t* config);

/** Writes len bytes of the given text to the control center. */
void protocol_control_center_write(char* text, size_t len);

/** Reads at maximum len bytes from control center to the given buffer. */
ssize_t protocol_control_center_receive(char* text, size_t len);

/** Terminates the connection to the control center. */
void protocol_control_center_terminate();

#endif /* INC_PROTOCOL_CONTROL_CENTER_H_ */
