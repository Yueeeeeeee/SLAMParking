/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *******************************************************************************/

#ifndef INC_PROTOCOL_CAN_H_
#define INC_PROTOCOL_CAN_H_

#include <stdint.h>
#include <stdbool.h>

struct protocol_can_config {
	uint8_t can_id_coordinator_clock;	// CAN ID of the logical clock message
	uint8_t can_id_global_shutdown;		// CAN ID of the global shutdown message
	uint8_t can_id_local_unit;			// CAN ID of the current local unit
};
typedef struct protocol_can_config protocol_can_config_t;

struct protocol_can_remote_unit_config {
	char* unit_name;
	uint8_t unit_id;
	bool alive;
};
typedef struct protocol_can_remote_unit_config protocol_can_remote_unit_config_t;

#endif /* INC_PROTOCOL_CAN_H_ */
