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
#include "af3_component.h"
#include "debugprint.h"
#include "timeutil.h"

static af3_component_send_remote_callback_t send_remote = NULL;
static uint64_t thread_sleep_time = 50;
static list_t* component_list; // list of af3_component_t

struct af3_port {
	// static members
	af3_component_t* component;
	uint16_t id;
	char* name;
	uint16_t type;
	af3_channel_t* input_channel;
	list_t* output_channel_list;
	bool needs_transmission;
	// variable members
	void* value;	// pointer to the value buffer of the port
	bool noval;		// flag indicating no value is stored in the port
	bool filled;	// flag indicating is filled or still waiting its current value
};

struct af3_channel {
	uint16_t id;
	char* name;
	af3_port_t* source;
	af3_port_t* target;
};

struct af3_component {
	// static members
	uint8_t id;
	char* name;
	bool weakly;
	list_t* input_port_list;
	list_t* output_port_list;
	af3_component_implementation_callback_t implementation;
	// variable members
	bool computed;
};


static inline bool out_of_memory(void* ptr) {
	if(ptr == NULL) {
		perror("Out of memory during malloc() in af3_component.c.");
		return true;
	}
	return false;
}

/*
 * Functions for creating the AF3 component architecture in memory.
 */
af3_component_t* af3_component_create(uint8_t id, char* name, bool weaklyCausal, af3_component_implementation_callback_t implementation) {
	af3_component_t* component = malloc(sizeof(af3_component_t));
	if(out_of_memory(component)) {
		return NULL;
	}
	component->id = id;
	component->name = name;
	component->weakly = weaklyCausal;
	component->input_port_list = list_util_create_list();
	component->output_port_list = list_util_create_list();
	component->implementation = implementation;
	component->computed = false;
	return component;
}

void af3_component_system_add(af3_component_t* component) {
	list_util_append(component_list, component);
}

af3_channel_t* af3_channel_create(uint16_t id, char* name, af3_port_t* source, af3_port_t* target) {
	af3_channel_t* channel = malloc(sizeof(af3_channel_t));
	if(out_of_memory(channel)) {
		return NULL;
	}
	channel->id = id;
	channel->name = name;
	channel->source = source;
	if(source != NULL) {
		list_util_append(source->output_channel_list, channel);
	}
	channel->target = target;
	if(target != NULL) {
		target->input_channel = channel;
	}
	return channel;
}

af3_port_t* af3_port_create(uint16_t id, char* name, uint16_t type, bool needs_transmission) {
	af3_port_t* port = malloc(sizeof(af3_port_t));
	if(out_of_memory(port)) {
		return NULL;
	}
	port->component = NULL;
	port->id = id;
	port->name = name;
	port->type = type;
	port->input_channel = NULL;
	port->output_channel_list = list_util_create_list();
	port->needs_transmission = needs_transmission;

	port->value = NULL;
	port->noval = true;
	port->filled = false;
	return port;
}

void af3_component_add_input(af3_component_t* component, af3_port_t* port) {
	if(component == NULL || component->input_port_list == NULL || port == NULL) {
		return;
	}
	list_util_append(component->input_port_list, port);
	port->component = component;
}

void af3_component_add_output(af3_component_t* component, af3_port_t* port) {
	if(component == NULL || component->output_port_list == NULL || port == NULL) {
		return;
	}
	list_util_append(component->output_port_list, port);
	port->component = component;
}

/*
 * Functions for accessing attributes of component, port, and channels.
 */
uint8_t af3_component_get_id(af3_component_t* component) {
	return component->id;
}

bool af3_component_is_weakly_causal(af3_component_t* component) {
	return component->weakly;
}

char* af3_component_port_get_name(af3_port_t* port) {
	return port->name;
}

uint16_t af3_component_port_get_type(af3_port_t* port) {
	return port->type;
}

static af3_port_t* find_port(af3_component_t* component, list_t* port_list, uint16_t id) {
	af3_port_t* result = NULL;
	list_iterator_t* iter = list_util_iterator(port_list, true);
	while(list_util_iterator_has_next(iter)) {
		af3_port_t* port = (af3_port_t*)list_util_iterator_next(iter);
		if(port != NULL && port->component == component && port->id == id) {
			result = port;
			break;
		}
	}
	list_util_iterator_destroy(iter);
	return result;
}

af3_port_t* af3_component_find_input_port_by_id(af3_component_t* component, uint16_t id) {
	return find_port(component, component->input_port_list, id);
}

af3_port_t* af3_component_find_output_port_by_id(af3_component_t* component, uint16_t id) {
	return find_port(component, component->output_port_list, id);
}

list_iterator_t* af3_component_get_outgoing_channels_iterator(af3_port_t* port) {
	return list_util_iterator(port->output_channel_list, true);
}

/*
 * Functions for running the AF3 component architecture.
 */
bool af3_component_port_is_noval(af3_port_t* port) {
	if(port->input_channel != NULL && port->input_channel->source != NULL) {
		return af3_component_port_is_noval(port->input_channel->source);
	}
	return port->noval;
}

void* af3_component_port_get_value(af3_port_t* port) {
	if(port->input_channel != NULL && port->input_channel->source != NULL) {
		return af3_component_port_get_value(port->input_channel->source);
	}
	return port->value;
}

void af3_component_port_set_value(af3_port_t* port, void* value) {
	if(port->input_channel != NULL && port->input_channel->source != NULL) {
		af3_component_port_set_value(port->input_channel->source, value);
		return;
	}
	port->noval = (value != NULL);
	if(port->value != NULL) {
		af3_get_free_value_callback(port->value);
	}
	port->value = value;
	port->filled = true;
}

bool af3_component_port_is_filled(af3_port_t* port) {
	if(port->input_channel != NULL && port->input_channel->source != NULL) {
		return af3_component_port_is_filled(port->input_channel->source);
	}
	return port->filled;
}

void af3_component_port_empty(af3_port_t* port) {
	if(port->input_channel != NULL && port->input_channel->source != NULL) {
		af3_component_port_empty(port->input_channel->source);
		return;
	}
	port->filled = false;
}

void af3_component_empty_outputs(af3_component_t* component) {
	list_iterator_t* iter = list_util_iterator(component->output_port_list, true);
	while (list_util_iterator_has_next(iter)) {
		af3_port_t* port = (af3_port_t*) list_util_iterator_next(iter);
		af3_component_port_empty(port);
	}
	list_util_iterator_destroy(iter);
}

/*
 * Functions for running the AF3 component architecture.
 */
static bool af3_component_is_ready_to_run(af3_component_t* component) {
	list_iterator_t* iter = list_util_iterator(component->input_port_list, true);
	bool result = true;
	while(list_util_iterator_has_next(iter)) {
		af3_port_t* port = (af3_port_t*)list_util_iterator_next(iter);
		if(!af3_component_port_is_filled(port)) {
			result = false;
			break;
		}
	}
	list_util_iterator_destroy(iter);
	return result;
}

static void af3_component_flush_outputs(af3_component_t* component) {
	if(component == NULL || component->output_port_list == NULL || send_remote == NULL) {
		return;
	}
	list_iterator_t* iter = list_util_iterator(component->output_port_list, true);
	if(out_of_memory(iter)) {
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_MANY, "Flushing outputs of %s to remote.\n", component->name);
	while(list_util_iterator_has_next(iter)) {
		af3_port_t* port = (af3_port_t*)list_util_iterator_next(iter);
		if(af3_component_port_is_filled(port) && port->needs_transmission) {
			send_remote(component->id, port->id, port->type, port->value);
		}
		af3_component_port_empty(port);
	}
	list_util_iterator_destroy(iter);
}

static void af3_component_pre_execute(af3_component_t* component) {
	component->computed = false;
	if(!component->weakly) {
		af3_component_flush_outputs(component);
	} else {
		af3_component_empty_outputs(component);
	}
}

/** Executes the component and sends weakly causal signals. */
static bool af3_component_execute(af3_component_t* component) {
	if(component->computed) {
		return true;
	}
	if(af3_component_is_ready_to_run(component)) {
		debug_print(DEBUG_PRINT_LEVEL_MANY, "Running %s.\n", component->name);
		component->implementation(component);
		component->computed = true;
		if(component->weakly) {
			af3_component_flush_outputs(component);
		}
		return true;
	}
	return false;
}

void af3_component_run_system() {
	list_iterator_t* iterator = list_util_iterator(component_list, true);
	if(iterator == NULL) {
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_MANY, "Running pre-execution phase.\n");
	while(list_util_iterator_has_next(iterator)) {
		af3_component_t* component = (af3_component_t*)list_util_iterator_next(iterator);
		if(component != NULL) {
			af3_component_pre_execute(component);
		}
	}
	debug_print(DEBUG_PRINT_LEVEL_MANY, "Running execution phase.\n");
	bool finished = false;
	while(!finished) {
		finished = true;
		list_util_iterator_reset(iterator, true);
		while(list_util_iterator_has_next(iterator)) {
			af3_component_t* component = (af3_component_t*)list_util_iterator_next(iterator);
			if(component != NULL) {
				finished = af3_component_execute(component) && finished;
			}
		}
		if(!finished) {
			time_util_sleep_micros(thread_sleep_time);
		}
	}
	list_util_iterator_destroy(iterator);
}

bool af3_component_module_initialize(
		uint64_t tsl,
		af3_component_send_remote_callback_t send_remote_callback) {
	thread_sleep_time = tsl;
	send_remote = send_remote_callback;
	component_list = list_util_create_list();
	if(out_of_memory(component_list)) {
		return false;
	}
	return true;
}
