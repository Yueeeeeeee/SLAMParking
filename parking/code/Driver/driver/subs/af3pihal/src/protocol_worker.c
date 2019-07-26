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
#include "protocol_worker.h"

#include <stdbool.h>
#include <pthread.h>

#include "canthread.h"
#include "debugprint.h"
#include "timeutil.h"

#define TWICE (uint64_t)2

static protocol_can_config_t* can_configuration;
static protocol_worker_clock_config_t* clock_configuration;
static protocol_worker_computation_config_t* computation_configuration;

static uint64_t local_logical_clock;
static uint64_t coordinator_logical_clock;

static bool terminate = false;

static void can_message_receive(can_thread_message_t* message) {
	if (message->message_id == can_configuration->can_id_global_shutdown) {
		terminate = true;
		return;
	}
	if (message->message_id == can_configuration->can_id_coordinator_clock) {
		coordinator_logical_clock = can_thread_message_get(message);
		debug_print(DEBUG_PRINT_LEVEL_MANY,
				"Received coordinator clock %llu.\n",
				coordinator_logical_clock);
	}
}

static void phase_initialization() {
	debug_print(DEBUG_PRINT_LEVEL_MANY, "Initializing worker unit %s.\n",
			computation_configuration->name);
	terminate = false;
	coordinator_logical_clock = 0;
	local_logical_clock = 1;
}

static bool phase_synchronization() {
	debug_print(DEBUG_PRINT_LEVEL_MANY,
			"Worker unit %s waiting for coordinator clock message with ID %x.\n",
			computation_configuration->name,
			can_configuration->can_id_coordinator_clock);
	uint64_t send_timer = 200 * MILLIS_IN_MICROS;
	// initially waiting for coordinator clock while sending worker done
	while (coordinator_logical_clock != 1) {
		if (terminate) {
			return false;
		}
		if(send_timer >= 200 * MILLIS_IN_MICROS) {
			can_thread_message_t* message = can_thread_message_create(
					can_configuration->can_id_local_unit, 0);
			can_thread_message_set(message, 0);
			can_thread_message_post(message);
			send_timer -= 200 * MILLIS_IN_MICROS;
		}
		time_util_sleep_micros(clock_configuration->waiting_sleep_in_micros);
		send_timer += clock_configuration->waiting_sleep_in_micros;
	}
	return true;
}

static void perform_work() {
	if (computation_configuration->worker != NULL) {
		debug_print(DEBUG_PRINT_LEVEL_MANY, "Executing worker %s.\n",
				computation_configuration->name);
		computation_configuration->worker();
		debug_print(DEBUG_PRINT_LEVEL_MANY, "Worker %s finished.\n",
				computation_configuration->name);
	} else {
		debug_print(DEBUG_PRINT_LEVEL_MANY,
				"Worker %s simulation. %llu millis workload elapse ...\n",
				computation_configuration->name,
				computation_configuration->simulated_workload_in_millis);
		time_util_sleep_millis(
				computation_configuration->simulated_workload_in_millis);
	}
}

static bool coordinator_clock_barrier() {
	while (local_logical_clock == coordinator_logical_clock + 1) {
		if (terminate) {
			return false;
		}
		time_util_sleep_micros(clock_configuration->waiting_sleep_in_micros);
	}
	return local_logical_clock == coordinator_logical_clock;
}

static void phase_operation() {
	debug_print(DEBUG_PRINT_LEVEL_MANY,
			"Worker unit %s starting operation loop.\n",
			computation_configuration->name);
	// start worker main loop
	while (local_logical_clock != clock_configuration->stop_at_logical_time) {
		if (!coordinator_clock_barrier()) {
			if (terminate) {
				return;
			}
			debug_print(DEBUG_PRINT_LEVEL_FEW,
					"Coordinator clock synchronization lost (%llu <> %llu). Shutting down.\n",
					coordinator_logical_clock, local_logical_clock);
		}
		debug_print(DEBUG_PRINT_LEVEL_FEW,
				"****************************\n%llu: worker %s.\n****************************\n", local_logical_clock,
				computation_configuration->name);

		// compute application or simulate workload
		perform_work();

		// increase logical clock before sleeping
		local_logical_clock++;

		// wait for next coordinator clock
		debug_print(DEBUG_PRINT_LEVEL_MANY,
				"Worker %s waiting for coordinator clock.\n",
				computation_configuration->name);
	}
}

static void phase_termination() {
	debug_print(DEBUG_PRINT_LEVEL_MANY, "Worker unit %s terminating.\n",
			computation_configuration->name);
	// currently nothing to do here
}

int protocol_worker_main(protocol_can_config_t* can_config,
		protocol_worker_clock_config_t* clock_config,
		protocol_worker_computation_config_t* computation_config) {
	if (can_config == NULL || clock_config == NULL || computation_config == NULL) {
		perror("Illegal NULL argument in protocol_worker_main().\n");
		return -1;
	}
	// setup configuration variables
	can_configuration = can_config;
	clock_configuration = clock_config;
	computation_configuration = computation_config;

	can_thread_add_reception_listener(can_message_receive);

	debug_print(DEBUG_PRINT_LEVEL_FEW,
			"Starting %s with worker unit protocol.\n",
			computation_configuration->name);

	phase_initialization();
	if (!phase_synchronization()) {
		phase_termination();
		return -1;
	}
	phase_operation();
	phase_termination();
	return 0;
}
