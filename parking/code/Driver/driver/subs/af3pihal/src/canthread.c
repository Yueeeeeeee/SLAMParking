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
#include <canthread.h>
#include <pthread.h>
#include "debugprint.h"
#include "listutil.h"

static can_socket_t* can_socket;

static uint32_t sleep_time = 500;

static list_t* reception_listeners;

static pthread_t* sender_thread;
static pthread_mutex_t* sender_mutex;
static bool sender_abort = false;

static pthread_t* receiver_thread;
static bool receiver_abort = false;

struct can_thread_message_list {
	can_thread_message_t* message;
	struct can_thread_message_list* next;
};
typedef struct can_thread_message_list can_thread_message_list_t;

static can_thread_message_list_t* outbox_head;

static inline void out_of_memory() {
	perror("Out of memory during malloc().");
}

static bool outbox_empty() {
	bool result = true;
	if (pthread_mutex_lock(sender_mutex) == 0) {
		result = (outbox_head == NULL);
		if (pthread_mutex_unlock(sender_mutex) != 0) {
			perror("Mutex unlock error during can_thread_outbox_empty().");
		}
	} else {
		perror("Mutex lock error during can_thread_outbox_empty().");
	}
	return result;
}

static void enqueue_message(can_thread_message_t* message) {
	can_thread_message_list_t* new_list_entry = malloc(sizeof(can_thread_message_list_t));
	if (new_list_entry == NULL) {
		out_of_memory();
		return;
	}
	new_list_entry->message = message;
	new_list_entry->next = NULL;
	if (pthread_mutex_lock(sender_mutex) == 0) {
		if(outbox_head == NULL) {
			outbox_head = new_list_entry;
		} else {
			can_thread_message_list_t* cur = outbox_head;
			while(cur->next != NULL) {
				cur = cur->next;
			}
			cur->next = new_list_entry;
		}
		if (pthread_mutex_unlock(sender_mutex) != 0) {
			perror("Mutex unlock error during can_thread_enqueue_message().");
		}
	} else {
		perror("Mutex lock error during can_thread_enqueue_message().");
	}
}

static can_thread_message_t* dequeue_message() {
	can_thread_message_t* message = NULL;
	if (pthread_mutex_lock(sender_mutex) == 0) {
		can_thread_message_list_t* reclaim = outbox_head;
		if (reclaim != NULL) {
			message = reclaim->message;
			outbox_head = reclaim->next;
			free(reclaim);
		}
		if (pthread_mutex_unlock(sender_mutex) != 0) {
			perror("Mutex unlock error during can_thread_dequeue_message().");
		}
	} else {
		perror("Mutex lock error during can_thread_dequeue_message().");
	}
	return message;
}

static void* sender_thread_worker() {
	can_frame_t frame;
	while (!sender_abort || !outbox_empty()) {
		struct can_thread_message* message = dequeue_message();
		while (message != NULL) {
			frame.can_id = message->message_id;
			frame.can_dlc = 8;
			for(int i = 0; i < 8; i++) {
				frame.data[i] = message->data[i];
			}
			can_socket_send(can_socket, &frame);
			debug_print(DEBUG_PRINT_LEVEL_ALL, "libcanthread: message 0x%X sent: 0x%016llX.\n", frame.can_id, can_thread_message_get(message));
			free(message);
			message = dequeue_message();
		}
		usleep(sleep_time);
	}
	return NULL;
}

static inline void inform_reception_listeners(can_thread_message_t* msg) {
	list_iterator_t* iter = list_util_iterator(reception_listeners, true);
	while(list_util_iterator_has_next(iter)) {
		can_thread_reception_listener_t listener_receive = (can_thread_reception_listener_t)list_util_iterator_next(iter);
		listener_receive(msg);
	}
	list_util_iterator_destroy(iter);
}

static void* receiver_thread_worker() {
	can_frame_t frame;
	can_thread_message_t message;
	while (!receiver_abort) {
		if (can_socket_available(can_socket)) {
			int bytesReceived = can_socket_receive(can_socket, &frame);
			if (bytesReceived == LIBCANSOCKET_ERROR) {
				perror("Receiving frame failed during can_thread_receiver_thread_worker().\n");
				return NULL;
			}
			if (bytesReceived > 0 && !list_util_is_empty(reception_listeners)) {
				message.message_id = frame.can_id & 0xFF;
				for(int i = 0; i < 8; i++) {
					message.data[i] = frame.data[i];
				}
				inform_reception_listeners(&message);
				debug_print(DEBUG_PRINT_LEVEL_ALL, "libcanthread: message 0x%X received: 0x%016llX.\n", frame.can_id, can_thread_message_get(&message));
			}
		}
		usleep(sleep_time);
	}
	return NULL;
}

bool can_thread_create(const char* if_name) {
	can_socket = can_socket_open(if_name);
	if (can_socket == NULL) {
		return false;
	}
	debug_print(DEBUG_PRINT_LEVEL_MANY, "libcanthread: created can socket for %s.\n", if_name);

	sender_thread = malloc(sizeof(pthread_t));
	sender_mutex = malloc(sizeof(pthread_mutex_t));
	receiver_thread = malloc(sizeof(pthread_t));
	if (sender_thread == NULL || receiver_thread == NULL || sender_mutex == NULL) {
		out_of_memory();
		return false;
	}
	debug_print(DEBUG_PRINT_LEVEL_MANY, "libcanthread: allocated memory.\n");

	sender_abort = false;
	pthread_create(sender_thread, NULL, sender_thread_worker, NULL);
	pthread_mutex_init(sender_mutex, NULL);
	debug_print(DEBUG_PRINT_LEVEL_MANY, "libcanthread: sender thread created.\n");

	receiver_abort = false;
	pthread_create(receiver_thread, NULL, receiver_thread_worker, NULL);
	reception_listeners = list_util_create_list();
	debug_print(DEBUG_PRINT_LEVEL_MANY, "libcanthread: receiver thread created.\n");

	return true;
}

void can_thread_add_reception_listener(can_thread_reception_listener_t listener) {
	if(reception_listeners == NULL || listener == NULL) {
		return;
	}
	list_util_append(reception_listeners, listener);
}

can_thread_message_t* can_thread_message_create(uint8_t id, int vargc, ...) {
	va_list vargs;
	can_thread_message_t* message = malloc(sizeof(can_thread_message_t));
	if(message == NULL) {
		out_of_memory();
		return NULL;
	}

	message->message_id = id;
	va_start(vargs, vargc);
	if(vargc > 8) {
		vargc = 8; // limit message data length
	}
	for(int i=0; i < 8;i++) {
		if(i < vargc) {
			message->data[i] = va_arg(vargs, int) & 0xFF;
		} else {
			message->data[i] = 0x00;
		}
	}
	va_end(vargs);
	return message;
}

void can_thread_message_set(can_thread_message_t* message, uint64_t value) {
	if(message != NULL) {
		message->data[0] = value >> 56 & 0xFF;
		message->data[1] = value >> 48 & 0xFF;
		message->data[2] = value >> 40 & 0xFF;
		message->data[3] = value >> 32 & 0xFF;
		message->data[4] = value >> 24 & 0xFF;
		message->data[5] = value >> 16 & 0xFF;
		message->data[6] = value >> 8 & 0xFF;
		message->data[7] = value & 0xFF;
	}
}

uint64_t can_thread_message_get(can_thread_message_t* message) {
	uint64_t value = 0;
	if(message != NULL) {
		value |= (uint64_t)message->data[0] << 56;
		value |= (uint64_t)message->data[1] << 48;
		value |= (uint64_t)message->data[2] << 40;
		value |= (uint64_t)message->data[3] << 32;
		value |= (uint64_t)message->data[4] << 24;
		value |= (uint64_t)message->data[5] << 16;
		value |= (uint64_t)message->data[6] << 8;
		value |= (uint64_t)message->data[7];
	}
	return value;
}

void can_thread_set_thread_sleep_in_micros(uint32_t micros) {
	sleep_time = micros;
	debug_print(DEBUG_PRINT_LEVEL_ALL, "libcanthread: thread sleep time set to %u micro-seconds.\n", micros);
}

void can_thread_message_post(can_thread_message_t* message) {
	if (message == NULL) {
		return;
	}
	enqueue_message(message);
	debug_print(DEBUG_PRINT_LEVEL_ALL, "libcanthread: message 0x%X posted to outbox.\n", message->message_id);
}

void can_thread_terminate() {
	sender_abort = true;
	receiver_abort = true;
	pthread_join(*sender_thread, NULL);
	debug_print(DEBUG_PRINT_LEVEL_MANY, "libcanthread: sender thread terminated.\n");
	pthread_join(*receiver_thread, NULL);
	debug_print(DEBUG_PRINT_LEVEL_MANY, "libcanthread: receiver thread terminated.\n");
}
