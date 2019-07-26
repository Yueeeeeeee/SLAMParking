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

#ifndef INC_AF3_COMPONENT_H_
#define INC_AF3_COMPONENT_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "af3.h"
#include "listutil.h"

typedef struct af3_component af3_component_t;

typedef struct af3_port af3_port_t;

typedef struct af3_channel af3_channel_t;

/*
 * Function for initializing the AF3 component module.
 */
typedef void (*af3_component_send_remote_callback_t)(uint8_t unit_id, uint16_t signal_id, uint16_t type, void* value);
/** Initializes the component sub-system and returns true if all memory could be allocated. */
bool af3_component_module_initialize(
		uint64_t thread_sleep_time,
		af3_component_send_remote_callback_t send_remote_callback);
/*
 * Functions for creating the AF3 component architecture in memory.
 */
typedef void (*af3_component_implementation_callback_t)(af3_component_t* wrapper);
/** Creates a component with the given id, name, and causality flag. */
af3_component_t* af3_component_create(uint8_t id, char* name, bool weaklyCausal, af3_component_implementation_callback_t implementation);

/** Creates a channel for the given ports. */
af3_channel_t* af3_channel_create(uint16_t id, char* name, af3_port_t* source, af3_port_t* target);

/** Creates an AF3 port. */
af3_port_t* af3_port_create(uint16_t id, char* name, uint16_t type, bool needs_transmission);

/** Adds the given component to the logical system. */
void af3_component_system_add(af3_component_t* component);

/** Initializes an input port with the given settings and a value of NULL. */
void af3_component_add_input(af3_component_t* component, af3_port_t* port);

/** Initializes an output port with the given settings and a value of NULL. */
void af3_component_add_output(af3_component_t* component, af3_port_t* port);

/*
 * Functions for accessing attributes of component, port, and channels.
 */
/** Returns the component id. */
uint8_t af3_component_get_id(af3_component_t* component);

/** Returns whether the given component is weakly causal. */
bool af3_component_is_weakly_causal(af3_component_t* component);

/** Returns the name of the port. */
char* af3_component_port_get_name(af3_port_t* port);

/** Returns the type of the port. */
uint16_t af3_component_port_get_type(af3_port_t* port);

/** Returns the input port with the given id. */
af3_port_t* af3_component_find_input_port_by_id(af3_component_t* component, uint16_t id);

/** Returns the output port with the given id. */
af3_port_t* af3_component_find_output_port_by_id(af3_component_t* component, uint16_t id);

/** Returns the list forward iterator of outgoing channels. Callers must destroy the iterator after use. */
list_iterator_t* af3_component_get_outgoing_channels_iterator(af3_port_t* port);

/*
 * Functions for setting and accessing port values.
 */
/** Returns the value contained in the port or NULL if NoVal is contained. */
void* af3_component_port_get_value(af3_port_t* port);

/** Sets the port value. If value is NULL the NoVal flag is set instead. */
void af3_component_port_set_value(af3_port_t* port, void* value);

/** Empties the port. */
void af3_component_port_empty(af3_port_t* port);

/** Empties all output ports. */
void af3_component_empty_outputs(af3_component_t* component);

/** Returns whether the port is filled or empty. */
bool af3_component_port_is_filled(af3_port_t* port);

/*
 * Functions for running the AF3 component architecture.
 */
/** Runs the AF3 semantics using the given list of components. */
void af3_component_run_system();

#endif /* INC_AF3_COMPONENT_H_ */
