/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *    Constantin Dresel - added struct receive_message_t
 *******************************************************************************/
 
#ifndef INC_AF3_H_
#define INC_AF3_H_

#include <canthread.h>
#include <stdbool.h>
#include <stdint.h>

// Make sure "data_dictionary.h" is included before "autofocus3_layer.h" for generated code
#ifndef __HEADER_data_dictionary_h
typedef bool	GEN_TYPE_boolean;
typedef int		GEN_TYPE_int;
typedef double	GEN_TYPE_double;
#endif


// new structure for receiveing message
typedef struct receive_message_t{
	char modulename[30];
	char variablename[30];
	char value[30];
	char time[30];
}receive_message_t;

/*
 * Function for initializing the module and callbacks
 */
typedef void* (*af3_component_copy_value_callback_t)(uint16_t type, void* value);
typedef void (*af3_component_free_value_callback_t)(uint16_t type, void* value);
void af3_module_initialize(
		char* unit_identifier, uint16_t number_of_user_types,
		af3_component_copy_value_callback_t copy_value_callback,
		af3_component_free_value_callback_t free_value_callback);
af3_component_copy_value_callback_t af3_get_copy_value_callback();
af3_component_free_value_callback_t af3_get_free_value_callback();

/*
 * Functions for creating and reading CAN messages.
 */
/** Create a message containing a NoVal value for this unit and signal id. */
can_thread_message_t* af3_message_create_noval(uint8_t unit_id, uint16_t signal_id);
/** Create a message containing a boolean value for this unit and signal id. */
can_thread_message_t* af3_message_create_bool(uint8_t unit_id, uint16_t signal_id, GEN_TYPE_boolean* value);
/** Create a message containing an integer value for this unit and signal id. */
can_thread_message_t* af3_message_create_int(uint8_t unit_id, uint16_t signal_id, GEN_TYPE_int* value);
/** Create a message containing a double value for this unit and signal id. */
can_thread_message_t* af3_message_create_double(uint8_t unit_id, uint16_t signal_id, GEN_TYPE_double* value);
// TODO: other messages for composite types

/** Returns the unit id of the given message. */
uint8_t af3_message_get_unit_id(can_thread_message_t* message);
/** Returns the signal id of the given message. */
uint16_t af3_message_get_signal_id(can_thread_message_t* message);

/** Returns whether the message represents a NoVal value. */
GEN_TYPE_boolean af3_message_is_noval(can_thread_message_t* message);
/** Returns the boolean value contained in the message. */
GEN_TYPE_boolean af3_message_get_boolean(can_thread_message_t* message);
/** Returns the integer value contained in the message. */
GEN_TYPE_int af3_message_get_int(can_thread_message_t* message);
/** Returns the double value contained in the message. */
GEN_TYPE_double af3_message_get_double(can_thread_message_t* message);
/** Returns the raw payload value as an unsigned 4-byte integer. */
uint32_t af3_message_get_raw_value(can_thread_message_t* message);

/*
 * Functions for communicating with the control center application via TCP socket.
 */
/** Activates the control center connection. */
void af3_cc_activate(char* server_ip_address, uint16_t server_port);
/** Sends a NoVal signal with the given name to the control center. */
void af3_cc_send_noval(char* signal, uint64_t timestamp);
/** Sends a boolean signal with the given name to the control center. */
void af3_cc_send_boolean(char* signal, GEN_TYPE_boolean value, uint64_t timestamp);
/** Sends an int signal with the given name to the control center. */
void af3_cc_send_int(char* signal, GEN_TYPE_int value, uint64_t timestamp);
/** Sends a double signal with the given name to the control center. */
void af3_cc_send_double(char* signal, GEN_TYPE_double value, uint64_t timestamp);
/** Deactivates the control center connection. */
void af3_cc_deactivate();

#endif /* INC_AF3_H_ */
