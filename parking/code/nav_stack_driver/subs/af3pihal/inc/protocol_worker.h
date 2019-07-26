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
#ifndef INC_PROTOCOL_WORKER_H_
#define INC_PROTOCOL_WORKER_H_

#include <stdbool.h>
#include <stdint.h>

#include "listutil.h"
#include "protocol_can.h"

struct protocol_worker_computation_config {
	char* name;
	void(*worker)(void);
	uint64_t simulated_workload_in_millis;
};
typedef struct protocol_worker_computation_config protocol_worker_computation_config_t;

struct protocol_worker_clock_config {
	uint64_t cycle_timeout_in_millis;
	uint64_t stop_at_logical_time;
	uint64_t waiting_sleep_in_micros;
};
typedef struct protocol_worker_clock_config	protocol_worker_clock_config_t;

/** Main function of worker unit. */
int protocol_worker_main(
		protocol_can_config_t* can_config,
		protocol_worker_clock_config_t* clock_config,
		protocol_worker_computation_config_t* computation_config);

#endif /* INC_PROTOCOL_WORKER_H_ */
