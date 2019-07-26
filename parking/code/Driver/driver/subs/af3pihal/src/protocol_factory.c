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
#include "protocol_factory.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static bool malloc_null_check(void* ptr) {
	if(ptr == NULL) {
		perror("Out of memory during malloc().");
		return false;
	}
	return true;
}

protocol_can_remote_unit_config_t* protocol_remote_unit_config_create(char* name, uint8_t id) {
	protocol_can_remote_unit_config_t* result = malloc(sizeof(protocol_can_remote_unit_config_t));
	if(!malloc_null_check(result)) {
		return NULL;
	}
	result->unit_name = name;
	result->unit_id = id;
	result->alive = false;
	return result;
}

protocol_can_config_t* protocol_can_config_create(
	uint8_t can_id_coordinator_clock,
	uint8_t can_id_global_shutdown,
	uint8_t can_id_local_unit) {
	protocol_can_config_t* result = malloc(sizeof(protocol_can_config_t));
	if(!malloc_null_check(result)) {
		return NULL;
	}
	result->can_id_coordinator_clock = can_id_coordinator_clock;
	result->can_id_global_shutdown = can_id_global_shutdown;
	result->can_id_local_unit = can_id_local_unit;
	return result;
}


protocol_worker_clock_config_t* protocol_worker_clock_config_create(
		uint64_t cycle_timeout_in_millis,
		uint64_t stop_at_logical_time,
		uint64_t waiting_sleep_in_micros) {
	protocol_worker_clock_config_t* result = malloc(sizeof(protocol_worker_clock_config_t));
	if(!malloc_null_check(result)) {
		return NULL;
	}
	result->cycle_timeout_in_millis = cycle_timeout_in_millis;
	result->stop_at_logical_time = stop_at_logical_time;
	result->waiting_sleep_in_micros = waiting_sleep_in_micros;
	return result;
}

protocol_worker_computation_config_t* protocol_worker_computation_config_create(
	char* name,
	void(*worker)(void),
	uint64_t simulated_workload_in_millis) {
	protocol_worker_computation_config_t* result = malloc(sizeof(protocol_worker_computation_config_t));
	if(!malloc_null_check(result)) {
		return NULL;
	}
	result->name = name;
	result->worker = worker;
	result->simulated_workload_in_millis = simulated_workload_in_millis;
	return result;
}
