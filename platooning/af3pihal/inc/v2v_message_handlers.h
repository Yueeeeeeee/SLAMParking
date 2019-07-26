/** @file v2v_msg_handlers.h
 * \brief Handler functions for received vehicle 2 vehicle messages.
 *
 *  Created on: 13.05.2018
 *  Author: Constantin
 */

#ifndef V2V_MESSAGE_HANDLERS_H_
#define V2V_MESSAGE_HANDLERS_H_

#include "v2v_message_types.h"
#include "v2v_udp_basic.h"
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

/**
 * \brief Monitoring handler for unknown vehicle 2 vehicle message types.
 * This function accepts pointers to received messages of arbitrary types and
 * processes the message content for communication monitoring - i.e. it prints
 * the message content to the console in a userfriendly augmented formatting.
 * CAUTION: global_debug_print_level has to be set to DEBUG_PRINT_LEVEL_FEW or
 * something more verbose for this function to produce any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_unknown(uint8_t* p_msg, size_t msg_len);

/**
 * \brief Monitoring handler for the vehicle 2 vehicle message type PLATOON.
 * This function accepts pointers to received messages of the type PLATOON and
 * processes the message content for communication monitoring - i.e. it prints
 * the message content to the console in a userfriendly augmented formatting.
 * CAUTION: global_debug_print_level has to be set to DEBUG_PRINT_LEVEL_FEW or
 * something more verbose for this function to produce any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_platoon(uint8_t* p_msg, size_t msg_len);

/**
 * \brief Monitoring handler for the vehicle2vehicle msg type LEADER HEARTBEAT.
 * This function accepts pointers to received messages of the type LEADER
 * HEARTBEAT and processes the message content for communication monitoring -
 * i.e. it prints the message content to the console in a userfriendly augmented
 * formatting. CAUTION: global_debug_print_level has to be set to
 * DEBUG_PRINT_LEVEL_FEW or something more verbose for this function to produce
 * any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_leader_heartbeat(uint8_t* p_msg,
		size_t msg_len);

/**
 * \brief Monitoring handler for the vehicle 2 vehicle message type FOLLOWER.
 * This function accepts pointers to received messages of the type FOLLOWER and
 * processes the message content for communication monitoring - i.e. it prints
 * the message content to the console in a userfriendly augmented formatting.
 * CAUTION: global_debug_print_level has to be set to DEBUG_PRINT_LEVEL_FEW or
 * something more verbose for this function to produce any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_follower(uint8_t* p_msg, size_t msg_len);

/**
 * \brief Monitoring handler for the vehicle 2 vehicle msg type LEAVE PLATOON.
 * This function accepts pointers to received messages of the type LEAVE PLATOON
 * and processes the message content for communication monitoring - i.e. it
 * prints the message content to the console in a userfriendly augmented
 * formatting. CAUTION: global_debug_print_level has to be set to
 * DEBUG_PRINT_LEVEL_FEW or something more verbose for this function to produce
 * any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_leave_platoon(uint8_t* p_msg, size_t msg_len)
;

/**
 * \brief Monitoring handler for the vehicle 2 vehicle message type SPLIT.
 * This function accepts pointers to received messages of the type SPLIT and
 * processes the message content for communication monitoring - i.e. it prints
 * the message content to the console in a userfriendly augmented formatting.
 * CAUTION: global_debug_print_level has to be set to DEBUG_PRINT_LEVEL_FEW or
 * something more verbose for this function to produce any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_split(uint8_t* p_msg, size_t msg_len);

/**
 * \brief Monitoring handler for the vehicle 2 vehicle message type NEW LEADER.
 * This function accepts pointers to received messages of the type NEW LEADER
 * and processes the message content for communication monitoring - i.e. it
 * prints the message content to the console in a userfriendly augmented
 * formatting.CAUTION: global_debug_print_level has to be set to
 * DEBUG_PRINT_LEVEL_FEW or something more verbose for this function to produce
 * any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_new_leader(uint8_t* p_msg, size_t msg_len);

/**
 * \brief Monitoring handler for the vehicle 2 vehicle msg type FUSE PLATOON.
 * This function accepts pointers to received messages of the type FUSE PLATOON
 * and processes the message content for communication monitoring - i.e. it
 * prints the message content to the console in a userfriendly augmented
 * formatting.CAUTION: global_debug_print_level has to be set to
 * DEBUG_PRINT_LEVEL_FEW or something more verbose for this function to produce
 * any output.
 * @param p_msg A pointer to the received message.
 * @param msg_len The length of the message pointed to given in bytes.
 */
void v2v_msg_handler_debug_monitor_fuse_platoon(uint8_t* p_msg, size_t msg_len);

void v2v_msg_handler_debug_monitor_handshake(uint8_t* p_msg, size_t msg_len);

typedef struct v2v_msg_handler_list_entry_t {
	size_t actual_message_length;
	uint8_t is_new; //TODO remove when list exists
	uint8_t message[V2V_MAX_UDP_PAYLOAD_BYTES]; //TODO adapt len
}v2v_msg_handler_list_entry_t;

typedef struct v2v_msg_handler_buffers_t {
	v2v_msg_handler_list_entry_t platoon_buffer;
	v2v_msg_handler_list_entry_t leader_heartbeat_buffer;
	v2v_msg_handler_list_entry_t follower_buffer;
	v2v_msg_handler_list_entry_t leave_platoon_buffer;
	v2v_msg_handler_list_entry_t split_buffer;
	v2v_msg_handler_list_entry_t new_leader_buffer;
	v2v_msg_handler_list_entry_t fuse_platoon_buffer;
	v2v_msg_handler_list_entry_t handshake_buffer;
}v2v_msg_handler_buffers_t;

void v2v_msg_handler_setup(v2v_msg_handler_buffers_t* p_buffers,
		pthread_mutex_t* p_mutex);

void v2v_msg_handler_store_platoon(uint8_t* p_msg, size_t msg_len);
void v2v_msg_handler_store_leader_heartbeat(uint8_t* p_msg, size_t msg_len);
void v2v_msg_handler_store_follower(uint8_t* p_msg, size_t msg_len);
void v2v_msg_handler_store_leave_platoon(uint8_t* p_msg, size_t msg_len);
void v2v_msg_handler_store_split(uint8_t* p_msg, size_t msg_len);
void v2v_msg_handler_store_new_leader(uint8_t* p_msg, size_t msg_len);
void v2v_msg_handler_store_fuse_platoon(uint8_t* p_msg, size_t msg_len);
void v2v_msg_handler_store_handshake(uint8_t* p_msg, size_t msg_len);

#endif /* V2V_MESSAGE_HANDLERS_H_ */
