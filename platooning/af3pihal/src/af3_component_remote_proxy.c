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
#include <af3_component_remote_proxy.h>
#include <pthread.h>
#include "debugprint.h"
#include "listutil.h"

static inline bool out_of_memory(void* ptr) {
	if(ptr == NULL) {
		perror("Out of memory during malloc() in af3_component.c.");
		return true;
	}
	return false;
}

static list_t* remote_proxies = NULL;	// list of af3_component_t
static list_iterator_t* remote_proxies_iterator = NULL;
static pthread_mutex_t* mutex = NULL;

bool af3_component_remote_proxy_module_initialize() {
	remote_proxies = list_util_create_list();
	if(out_of_memory(remote_proxies)) {
		return false;
	}
	remote_proxies_iterator = list_util_iterator(remote_proxies, true);
	if(out_of_memory(remote_proxies_iterator)) {
		return false;
	}
	mutex = malloc(sizeof(pthread_mutex_t));
	if(out_of_memory(mutex)) {
		return false;
	}
	pthread_mutex_init(mutex, NULL);
	return true;
}

static af3_component_t* get_or_create_remote_proxy(uint8_t remote_unit_id) {
	pthread_mutex_lock(mutex);
	list_util_iterator_reset(remote_proxies_iterator, true);
	while(list_util_iterator_has_next(remote_proxies_iterator)) {
		af3_component_t* component = (af3_component_t*)list_util_iterator_next(remote_proxies_iterator);
		if(af3_component_get_id(component) == remote_unit_id) {
			pthread_mutex_unlock(mutex);
			return component;
		}
	}
	af3_component_t* newProxy = af3_component_create(remote_unit_id, "", false, NULL);
	if(newProxy != NULL) {
		list_util_append(remote_proxies, newProxy);
	}
	pthread_mutex_unlock(mutex);
	return newProxy;
}

af3_port_t* af3_component_remote_proxy_input_create(uint8_t remote_unit_id, uint16_t signal_id, char* name, uint16_t type) {
	af3_component_t* remote_unit_proxy = get_or_create_remote_proxy(remote_unit_id);
	af3_port_t* port = af3_port_create(signal_id, name, type, false);
	if(out_of_memory(port)) {
		return NULL;
	}
	af3_component_add_output(remote_unit_proxy, port);
	return port;
}

bool af3_component_remote_proxy_handle_input(uint8_t remote_unit_id, uint16_t signal_id, void* value) {
	af3_component_t* proxy = get_or_create_remote_proxy(remote_unit_id);
	if(proxy == NULL) {
		return false;
	}
	af3_port_t* port = af3_component_find_output_port_by_id(proxy, signal_id);
	if(port == NULL) {
		return false;
	}
	debug_print(DEBUG_PRINT_LEVEL_MANY, "Setting unit 0x%X signal 0x%X.\n", remote_unit_id, signal_id);
	af3_component_port_set_value(port, value);
	return true;
}

void af3_component_remote_proxy_clear_inputs() {
	pthread_mutex_lock(mutex);
	list_util_iterator_reset(remote_proxies_iterator, true);
	while(list_util_iterator_has_next(remote_proxies_iterator)) {
		af3_component_t* component = (af3_component_t*)list_util_iterator_next(remote_proxies_iterator);
		af3_component_empty_outputs(component);
	}
	pthread_mutex_unlock(mutex);
}
