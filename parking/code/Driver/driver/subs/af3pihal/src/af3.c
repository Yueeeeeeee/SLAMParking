/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *    Constantin Dresel - integration of downstream functions
 *******************************************************************************/
#include "af3.h"
#include "control_center_downstream.h"
#include "protocol_control_center.h"
#include <pthread.h>
#include <stdio.h>

/*
 * Function for initializing the module and callbacks
 */
static char* execution_unit_identifier = NULL;
static af3_component_copy_value_callback_t copy_value = NULL;
static af3_component_free_value_callback_t free_value = NULL;
void af3_module_initialize(char* unit_identifier,
		uint16_t number_of_user_types,
		af3_component_copy_value_callback_t copy_value_callback,
		af3_component_free_value_callback_t free_value_callback) {
	execution_unit_identifier = unit_identifier;
	copy_value = copy_value_callback;
	free_value = free_value_callback;
}

af3_component_copy_value_callback_t af3_get_copy_value_callback() {
	return copy_value;
}
af3_component_free_value_callback_t af3_get_free_value_callback() {
	return free_value;
}

/*
 * Functions for creating and reading CAN messages.
 */
can_thread_message_t* af3_message_create_noval(uint8_t unit_id, uint16_t signal_id) {
	return can_thread_message_create(unit_id, 8,
			(signal_id >> 8) & 0xFF, signal_id & 0xFF, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00);
}
can_thread_message_t* af3_message_create_bool(uint8_t unit_id, uint16_t signal_id, GEN_TYPE_boolean* value) {
	return can_thread_message_create(unit_id, 8,
			(signal_id >> 8) & 0xFF, signal_id & 0xFF, 1, 1,
			*value ? 0x01: 0x00, 0x00, 0x00, 0x00);
}
can_thread_message_t* create_message_uint32(uint8_t unit_id, uint16_t signal_id, uint32_t value) {
	return can_thread_message_create(unit_id, 8,
			(signal_id >> 8) & 0xFF, signal_id & 0xFF, 1, 1,
			(value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF);
}
can_thread_message_t* af3_message_create_int(uint8_t unit_id, uint16_t signal_id, GEN_TYPE_int* value) {
	return create_message_uint32(unit_id, signal_id, (uint32_t)*value);
}
can_thread_message_t* af3_message_create_double(uint8_t unit_id, uint16_t signal_id, GEN_TYPE_double* value) {
	return create_message_uint32(unit_id, signal_id, (uint32_t)*value);
}

uint8_t af3_message_get_unit_id(can_thread_message_t* message) {
	return message->message_id;
}
uint16_t af3_message_get_signal_id(can_thread_message_t* message) {
	return (uint16_t)message->data[0] << 8 | (uint16_t)message->data[1];
}

GEN_TYPE_boolean af3_message_is_noval(can_thread_message_t* message) {
	if(message == NULL) {
		return true;
	}
	return message->data[2] == 0x00 && message->data[3] == 0x00;
}
GEN_TYPE_boolean af3_message_get_boolean(can_thread_message_t* message) {
	return message->data[4] == 0x01;
}
static uint32_t get_uint32(can_thread_message_t* message) {
	uint32_t result = 0;
	result |= message->data[4] << 24;
	result |= message->data[5] << 16;
	result |= message->data[6] << 8;
	result |= message->data[7];
	return result;
}
GEN_TYPE_int af3_message_get_int(can_thread_message_t* message) {
	return (int)get_uint32(message);
}
GEN_TYPE_double af3_message_get_double(can_thread_message_t* message) {
	return (double)get_uint32(message);
}

uint32_t af3_message_get_raw_value(can_thread_message_t* message) {
	return get_uint32(message);
}

/*
 * Functions for communicating with the control center application via TCP socket.
 */
static char* output_buffer = NULL;
static size_t output_buffer_size = 0;
static pthread_mutex_t* output_mutex = NULL;
static void ensure_output_buffer_size(size_t size) {
	// assumes that output_mutex is locked
	if(size > output_buffer_size) {
		if(output_buffer != NULL) {
			free(output_buffer);
		}
		output_buffer = malloc(size);
		output_buffer_size = size;
	}
}

void af3_cc_activate(char* server_ip_address, uint16_t server_port) {
	// setup connection to control center
	protocol_control_center_configuration_t cc_config;
	cc_config.server_ip_address = server_ip_address;
	cc_config.server_port = server_port;
	protocol_control_center_create(&cc_config);

	output_mutex = malloc(sizeof(pthread_mutex_t));
	pthread_mutex_init(output_mutex, NULL);
	ensure_output_buffer_size(128);

	//initialize downstream communication
	af3_cc_donwstream_init();

}
void af3_cc_send_noval(char* signal, uint64_t timestamp) {
	pthread_mutex_lock(output_mutex);
	int needed = snprintf(output_buffer, output_buffer_size, "%s#%s=@%016llX\n", execution_unit_identifier, signal, timestamp);
	if(needed > output_buffer_size) {
		ensure_output_buffer_size(needed + 1);
		snprintf(output_buffer, output_buffer_size, "%s#%s=@%016llX\n", execution_unit_identifier, signal, timestamp);
	}
	protocol_control_center_write(output_buffer, needed);
	pthread_mutex_unlock(output_mutex);
}
void af3_cc_send_boolean(char* signal, GEN_TYPE_boolean value, uint64_t timestamp) {
	pthread_mutex_lock(output_mutex);
	int needed = snprintf(output_buffer, output_buffer_size, "%s#%s=%s@%016llX\n", execution_unit_identifier, signal, value?"true":"false", timestamp);
	if(needed > output_buffer_size) {
		ensure_output_buffer_size(needed + 1);
		snprintf(output_buffer, output_buffer_size, "%s#%s=%s@%016llX\n", execution_unit_identifier, signal, value?"true":"false", timestamp);
	}
	protocol_control_center_write(output_buffer, needed);
	pthread_mutex_unlock(output_mutex);
}
void af3_cc_send_int(char* signal, GEN_TYPE_int value, uint64_t timestamp) {
	pthread_mutex_lock(output_mutex);
	int needed = snprintf(output_buffer, output_buffer_size, "%s#%s=%08X@%016llX\n", execution_unit_identifier, signal, value, timestamp);
	if(needed > output_buffer_size) {
		ensure_output_buffer_size(needed + 1);
		snprintf(output_buffer, output_buffer_size, "%s#%s=%08X@%016llX\n", execution_unit_identifier, signal, value, timestamp);
	}
	protocol_control_center_write(output_buffer, needed);
	pthread_mutex_unlock(output_mutex);
}
void af3_cc_send_double(char* signal, GEN_TYPE_double value, uint64_t timestamp) {
	pthread_mutex_lock(output_mutex);
	int needed = snprintf(output_buffer, output_buffer_size, "%s#%s=%f@%016llX\n", execution_unit_identifier, signal, value, timestamp);
	if(needed > output_buffer_size) {
		ensure_output_buffer_size(needed + 1);
		snprintf(output_buffer, output_buffer_size, "%s#%s=%f@%016llX\n", execution_unit_identifier, signal, value, timestamp);
	}
	protocol_control_center_write(output_buffer, needed);
	pthread_mutex_unlock(output_mutex);
}

// downstream functionalities are located in control_center_downstream.h

void af3_cc_deactivate() {
	output_buffer_size = 0;
	if(output_buffer != NULL) {
		free(output_buffer);
	}
	if(output_mutex != NULL) {
		pthread_mutex_destroy(output_mutex);
		free(output_mutex);
	}
	af3_cc_downstream_exit();
	protocol_control_center_terminate();
}
