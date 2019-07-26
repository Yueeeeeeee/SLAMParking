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
#ifndef __CANTHREAD_H
#define __CANTHREAD_H

#include <cansocket.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/** The message structure for CAN threads. */
struct can_thread_message {
	uint8_t message_id;
	uint8_t data[8];
};
typedef struct can_thread_message can_thread_message_t;

/**
 * Callback function type for CAN message reception. Note that the pointer to
 * the message is allocated to the stack and MUST NOT be freed by the listener.
 */
typedef void (*can_thread_reception_listener_t)(can_thread_message_t* message);

/** Creates a sender thread and a receiver thread for the given interface. */
bool can_thread_create(const char* if_name);

/** Adds a reception listener to the retrieval list. */
void can_thread_add_reception_listener(can_thread_reception_listener_t listener);

/**
 * Creates a can message to be posted to the sender thread. Only up to
 * 8 bytes from the variable argument list are used.
 */
can_thread_message_t* can_thread_message_create(uint8_t id, int vargsc, ...);

/** Fills the message with the 64-bit int value (big-endian encoding). */
void can_thread_message_set(can_thread_message_t* message, uint64_t value);

/** Returns the 64-bit int value extracted from the message. */
uint64_t can_thread_message_get(can_thread_message_t* message);

/** Set the sleep time for sender and receiver threads. */
void can_thread_set_thread_sleep_in_micros(uint32_t micros);

/** Posts a message via the sender thread. */
void can_thread_message_post(can_thread_message_t* message);

/** Terminate the can threads and wait until they have shut down. */
void can_thread_terminate();

#endif // __CANTHREAD_H
