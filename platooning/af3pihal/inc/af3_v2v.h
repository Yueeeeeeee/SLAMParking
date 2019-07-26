/**
 * @file af3_v2v.h
 * \brief AF3 abstraction layer for UDP vehicle 2 vehicle communication.
 *
 *	Basically mainly a bunch of wrappers.
 *  Created on: 12.05.2018
 *  Author: Constantin
 */

#ifndef AF3_V2V_H_
#define AF3_V2V_H_

#include "v2v_message_types.h"
#include "af3.h"
#include <stdint.h>
#include <stdbool.h>

/** the number of received messages storable in the message read buffer at worst
 * case */
#define AF3_V2V_RX_BUFFER_MSG_CAPACITY 5

/** \brief Generic vehicle to vehicle message type for pointer creation.
 * Currently just contains message type Identifier and first message byte for
 * pointing pointers of this type to memory buffers.
 */
typedef struct af3_v2v_generic_message_t {
	uint8_t type_id;
	GEN_TYPE_int source_uid;
	size_t payload_length;
	uint8_t actual_message_start;
}__attribute__ ((packed)) af3_v2v_generic_message_t;

/**
 * Type for function pointer to v2v message handler for type PLATOON.
 * @param p_msg A pointer to the message to be handled.
 * @param msg_payload_len The length of the message to be handled given in byte.
 */
typedef void (*v2v_msg_platoon_handler_t) (uint8_t* p_msg,
		size_t msg_payload_len);
typedef void (*v2v_msg_leader_heartbeat_handler_t) (uint8_t* p_msg,
		size_t msg_payload_len);
typedef void (*v2v_msg_follower_handler_t) (uint8_t* p_msg,
		size_t msg_payload_len);
typedef void (*v2v_msg_leave_platoon_handler_t) (uint8_t* p_msg,
		size_t msg_payload_len);
typedef void (*v2v_msg_split_handler_t) (uint8_t* p_msg, size_t msg_payload_len)
		;
typedef void (*v2v_msg_new_leader_handler_t) (uint8_t* p_msg,
		size_t msg_payload_len);
typedef void (*v2v_msg_fuse_platoon_handler_t) (uint8_t* p_msg,
		size_t msg_payload_len);
typedef void (*v2v_msg_handshake_handler_t) (uint8_t* p_msg,
		size_t msg_payload_len);

/**
 * A strut for passing message handler pointers to functions.
 */
typedef struct af3_v2v_handler_functions_container_t {
	v2v_msg_platoon_handler_t platoon_handler;
	v2v_msg_leader_heartbeat_handler_t leader_heartbeat_handler;
	v2v_msg_follower_handler_t follower_handler;
	v2v_msg_leave_platoon_handler_t leave_platoon_handler;
	v2v_msg_split_handler_t split_handler;
	v2v_msg_new_leader_handler_t new_leader_handler;
	v2v_msg_fuse_platoon_handler_t fuse_platoon_handler;
	v2v_msg_handshake_handler_t handshake_handler;
}af3_v2v_handler_functions_container_t;

/**
 * \brief Call this to initialize vehicle 2 vehicle communication.
 * Sets up addresses, creates and binds socket to be able to do UDP broadcast
 * communication.
 * @param port The network port to be used for message transmission - value
 * doesn't matter for pure rx action.
 * @param local_uid A unique id for the local device.
 * @return The socket file descriptor on success a negative value otherwise.
 */
int af3_v2v_activate(uint16_t port, GEN_TYPE_int local_uid, bool start_lsitener)
;

/**
 * \brief Call this to exit vehicle 2 vehicle communication.
 * Closes the socket used for UDP vehicle 2 vehicle communication.
 * @return Zero on success, negative otherwise.
 */
int af3_v2v_deactivate(void);

/**
 * \brief Send a given vehicle 2 vehicle message.
 * Transmits a given vehicle 2 vehicle message of given type as UDP broadcast
 * packet to other devices on the network.
 * @param msg_type The type identifier of the message to be transmitted.
 * @param p_msg A pointer to the first Byte of the actual message to be
 * transmitted.
 * @param msg_len The length of the message to send given in bytes.
 * @return The number of bytes transmitted on success, negative otherwise.
 */
ssize_t af3_v2v_send_msg(uint8_t msg_type, uint8_t* p_msg, size_t msg_len);

/**
 * \brief Wait for incoming messages and pass them to specified handlers.
 * Waits for incoming vehicle 2 vehicle messages and them to specified handler
 * functions. This function does usually not exit but sits and waits for UDP
 * messages when idle.
 */
// TODO comment
void* _af3_v2v_receive_msgs_thread_func(void* p_message_handlers);
//TODO comment
void af3_v2v_listen(af3_v2v_handler_functions_container_t* p_message_handlers);

/**
 * \brief Set the uid for the leader vehicle to listen to.
 * Sets the uid for the leader vehicle to listen to to the given value. This uid
 * e.g. is used for filtering incoming messages.
 * @param uid The unique ID of the leader vehicle to listen to.
 * @param uid_noval Indicator whether the given uid is valid or unset.
 */
void af3_v2v_set_leader_vehicle_of_interest_uid(GEN_TYPE_int uid,
		GEN_TYPE_boolean uid_noval);

/**
 * \brief Set the uid for the preceding vehicle to listen to.
 * Sets the uid for the vehicle directly preceding the ego vehicle to the given
 * value. This uid e.g. is used for filtering incoming messages.
 * @param uid The unique ID of the directly preceding vehicle to listen to.
 * @param uid_noval Indicator whether the given uid is valid or unset.
 */
void af3_v2v_set_preceding_vehicle_of_interest_uid(GEN_TYPE_int uid,
		GEN_TYPE_boolean uid_noval);

void af3_v2v_get_platoon(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);
void af3_v2v_get_leader_heartbeat(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);
void af3_v2v_get_follower(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);
void af3_v2v_get_leave_platoon(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);
void af3_v2v_get_split(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);
void af3_v2v_get_new_leader(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);
void af3_v2v_get_fuse_platoon(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);
void af3_v2v_get_handshake(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size);

void af3_v2v_send_platoon(uint8_t* p_src_struct, size_t src_size);
void af3_v2v_send_leader_heartbeat(uint8_t* p_src_struct, size_t src_size);
void af3_v2v_send_follower(uint8_t* p_src_struct, size_t src_size);
void af3_v2v_send_leave_platoon(uint8_t* p_src_struct, size_t src_size);
void af3_v2v_send_split(uint8_t* p_src_struct, size_t src_size);
void af3_v2v_send_new_leader(uint8_t* p_src_struct, size_t src_size);
void af3_v2v_send_fuse_platoon(uint8_t* p_src_struct, size_t src_size);
void af3_v2v_send_handshake(uint8_t* p_src_struct, size_t src_size);

#endif /* AF3_V2V_H_ */
