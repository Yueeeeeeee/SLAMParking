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

#ifndef INC_PROTOCOL_FACTORY_H_
#define INC_PROTOCOL_FACTORY_H_

#include "protocol_can.h"
#include "protocol_worker.h"
#include "protocol_coordinator.h"

/** Creates a remote unit configuration. */
protocol_can_remote_unit_config_t* protocol_remote_unit_config_create(char* name, uint8_t id);

/** Creates a CAN configuration. */
protocol_can_config_t* protocol_can_config_create(
	uint8_t can_id_coordinator_clock,
	uint8_t can_id_global_shutdown,
	uint8_t can_id_local_unit);

/** Creates a worker clock configuration. */
protocol_worker_clock_config_t* protocol_worker_clock_config_create(
		uint64_t cycle_timeout_in_millis,
		uint64_t stop_at_logical_time,
		uint64_t waiting_sleep_in_micros);

/** Creates a worker computation configuration. */
protocol_worker_computation_config_t* protocol_worker_computation_config_create(
	char* name,
	void(*worker)(void),
	uint64_t simulated_workload_in_millis);

#endif /* INC_PROTOCOL_FACTORY_H_ */
