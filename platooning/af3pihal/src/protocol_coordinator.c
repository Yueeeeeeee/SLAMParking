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
#include "protocol_coordinator.h"

#include <stdbool.h>
#include <pthread.h>

#include "canthread.h"
#include "debugprint.h"
#include "timeutil.h"

static protocol_can_config_t* can_configuration;
static protocol_worker_clock_config_t* clock_configuration;
static protocol_worker_computation_config_t* computation_configuration;

static list_t* remote_unit_list; //  list of protocol_remote_unit_config_t*
static pthread_mutex_t* remote_unit_list_mutex;

static uint64_t start_time_in_micros;
static uint64_t delta_time_in_micros;
static uint64_t local_logical_clock;

static bool terminate = false;

static void can_message_receive(can_thread_message_t* message) {
	if (message->message_id == can_configuration->can_id_global_shutdown) {
		terminate = true;
		return;
	}
	list_iterator_t* iter = list_util_iterator(remote_unit_list, true);
	pthread_mutex_lock(remote_unit_list_mutex);
	while (list_util_iterator_has_next(iter)) {
		protocol_can_remote_unit_config_t* remote_unit =
				(protocol_can_remote_unit_config_t*) list_util_iterator_next(
						iter);
		if (remote_unit->unit_id == message->message_id) {
			remote_unit->alive = true;
			break;
		}
	}
	pthread_mutex_unlock(remote_unit_list_mutex);
	list_util_iterator_destroy(iter);
}

static void reset_remote_units_alive() {
	list_iterator_t* iter = list_util_iterator(remote_unit_list, true);
	pthread_mutex_lock(remote_unit_list_mutex);
	while (list_util_iterator_has_next(iter)) {
		protocol_can_remote_unit_config_t* remote_unit =
				(protocol_can_remote_unit_config_t*) list_util_iterator_next(
						iter);
		remote_unit->alive = false;
	}
	pthread_mutex_unlock(remote_unit_list_mutex);
	list_util_iterator_destroy(iter);
}

static void send_coordinator_clock() {
	can_thread_message_t* message;
	debug_print(DEBUG_PRINT_LEVEL_FEW,
			"Coordinator %s sending logical clock %llu (CAN id %u).\n",
			computation_configuration->name, local_logical_clock,
			can_configuration->can_id_coordinator_clock);
	message = can_thread_message_create(
			can_configuration->can_id_coordinator_clock, 0);
	can_thread_message_set(message, local_logical_clock);
	can_thread_message_post(message);
}

static void send_terminate() {
	can_thread_message_t* message;
	debug_print(DEBUG_PRINT_LEVEL_FEW,
			"Coordinator sending global shutdown (CAN id %u).\n",
			can_configuration->can_id_global_shutdown);
	message = can_thread_message_create(
			can_configuration->can_id_global_shutdown, 0);
	can_thread_message_post(message);
}

static void perform_work() {
	if (computation_configuration->worker != NULL) {
		computation_configuration->worker();
	} else {
		debug_print(DEBUG_PRINT_LEVEL_MANY,
				"Coordinator %s simulation. %llu millis workload elapse ...\n",
				computation_configuration->name,
				computation_configuration->simulated_workload_in_millis);
		time_util_sleep_millis(
				computation_configuration->simulated_workload_in_millis);
	}
}

static void phase_initialization() {
	reset_remote_units_alive();
	terminate = false;
	local_logical_clock = 1;
}

static bool remote_units_alive_barrier() {
	bool all_alive = false;
	list_iterator_t* iter = list_util_iterator(remote_unit_list, true);
	while (!all_alive) {
		all_alive = true;
		pthread_mutex_lock(remote_unit_list_mutex);
		while (all_alive && list_util_iterator_has_next(iter)) {
			protocol_can_remote_unit_config_t* unit =
					(protocol_can_remote_unit_config_t*) list_util_iterator_next(
							iter);
			all_alive = unit->alive;
		}
		pthread_mutex_unlock(remote_unit_list_mutex);
		if (!all_alive) {
			list_util_iterator_reset(iter, true);
			time_util_sleep_micros(
					clock_configuration->waiting_sleep_in_micros);
		}
		if (terminate) {
			break;
		}
	}
	list_util_iterator_destroy(iter);
	return !terminate;
}

static bool phase_synchronization() {
	// wait for workers to show up
	if (!remote_units_alive_barrier()) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,
				"Aborting coordinator during synchronization phase.\n");
		return false;
	}
	return true;
}

static void phase_operation() {
	// config->stop_after_logical_tick == 0 =>
	// 		run until uint64 overflows in 584 billion years
	while (local_logical_clock != clock_configuration->stop_at_logical_time) {
		start_time_in_micros = time_util_get_current_micros();
		send_coordinator_clock();
		debug_print(DEBUG_PRINT_LEVEL_FEW,
				"****************************\n%llu: coordinator %s.\n****************************\n", local_logical_clock,
				computation_configuration->name);
		// compute application or simulate workload
		perform_work();

		// wait for the remainder of the current cycle
		delta_time_in_micros = time_util_get_elapsed_micros_since(
				start_time_in_micros);
		// this is a HACK, since for some yet unknown reason the first round takes substantially longer
		if (delta_time_in_micros
				> clock_configuration->cycle_timeout_in_millis
						* MILLIS_IN_MICROS) {
			debug_print(DEBUG_PRINT_LEVEL_FEW,
					"Remaining cycle time negative! Deadline broken (took %lluus). Shutting down.\n", delta_time_in_micros);
			return;
		}
		uint64_t remaining_time_in_micros =
				clock_configuration->cycle_timeout_in_millis * MILLIS_IN_MICROS
						- delta_time_in_micros;
		debug_print(DEBUG_PRINT_LEVEL_MANY,
				"Coordinator sleeping for %llu micros.\n",
				remaining_time_in_micros);
		time_util_sleep_micros(remaining_time_in_micros);
		// increase logical clock
		local_logical_clock++;
		// increase start time
		// start_time_in_micros += clock_configuration->cycle_timeout_in_millis
		//		* MILLIS_IN_MICROS;
	}
}

static void phase_termination() {
	send_terminate();
}

int protocol_coordinator_main(protocol_can_config_t* can_config,
		protocol_worker_clock_config_t* clock_config,
		protocol_worker_computation_config_t* computation_config,
		list_t* remote_units) {
	if (can_config == NULL || clock_config == NULL || computation_config == NULL) {
		perror("Illegal NULL argument in protocol_coordinator_main().\n");
		return -1;
	}
	// setup configuration variables
	can_configuration = can_config;
	clock_configuration = clock_config;
	computation_configuration = computation_config;

	remote_unit_list = remote_units;
	remote_unit_list_mutex = malloc(sizeof(pthread_mutex_t));
	pthread_mutex_init(remote_unit_list_mutex, NULL);
	can_thread_add_reception_listener(can_message_receive);

	debug_print(DEBUG_PRINT_LEVEL_FEW,
			"Starting %s with coordinator unit protocol.\n",
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
