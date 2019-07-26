/*
 * af3_v2v.c
 *
 *  Created on: 12.05.2018
 *      Author: Constantin
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <stdbool.h>

#include <linux/if.h> //for ifreq struct

#include "v2v_message_handlers.h"
#include "v2v_udp_basic.h"
#include "v2v_message_types.h"
#include "af3_v2v.h"
#include "debugprint.h"

//#define is_it_me(remote,me) (remote==me)

struct {
	struct sockaddr_in local_addr_n_port;
	struct sockaddr_in remote_addr_n_port_tx;
	struct sockaddr_in remote_addr_n_port_rx;
	socklen_t actual_rx_addr_len;
	int bc_socket_fd;
	pthread_t listener_thread_id;
	volatile sig_atomic_t listener_thread_run;
	pthread_mutex_t tx_mutex;
	//message filtering
	volatile GEN_TYPE_int local_uid; /*!< wont change usually but still
									asynchronous r/w access form multiple
									threads may happen */
	volatile GEN_TYPE_int leader_vehicle_of_interest_uid;
	volatile GEN_TYPE_int preceding_vehicle_of_interest_uid;
	pthread_mutex_t uid_mutex;

	af3_v2v_handler_functions_container_t message_handlers;
	v2v_msg_handler_buffers_t message_buffers;
	pthread_mutex_t rx_buffer_mutex;
}af3_v2v;

//TODO SECTION UNDER CONSTRUCTION
//TODO comment
void print_remote_ip(void) {
	char buff[INET_ADDRSTRLEN];
	const char* p_addr_string  = inet_ntop(
			(int)af3_v2v.remote_addr_n_port_rx.sin_family,
			(void*) &(af3_v2v.remote_addr_n_port_rx.sin_addr),
			buff, af3_v2v.actual_rx_addr_len);
	if(p_addr_string != NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "V2V: Remote IP: %s\n",
				p_addr_string);
	}
	else {
		v2v_print_error_msg("to get or convert remote IP.");
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "V2V: rx_addr %0X\n"
			,af3_v2v.remote_addr_n_port_rx.sin_addr.s_addr);
}

void listener_start_monitor(void) {
	//setup handler functions
	af3_v2v.message_handlers.platoon_handler =
			v2v_msg_handler_debug_monitor_platoon;
	af3_v2v.message_handlers.leader_heartbeat_handler =
			v2v_msg_handler_debug_monitor_leader_heartbeat;
	af3_v2v.message_handlers.follower_handler =
			v2v_msg_handler_debug_monitor_follower;
	af3_v2v.message_handlers.leave_platoon_handler =
			v2v_msg_handler_debug_monitor_leave_platoon;
	af3_v2v.message_handlers.split_handler =
			v2v_msg_handler_debug_monitor_split;
	af3_v2v.message_handlers.new_leader_handler =
			v2v_msg_handler_debug_monitor_new_leader;
	af3_v2v.message_handlers.fuse_platoon_handler =
			v2v_msg_handler_debug_monitor_fuse_platoon;
	af3_v2v.message_handlers.handshake_handler =
			v2v_msg_handler_debug_monitor_handshake;
	af3_v2v_listen(&(af3_v2v.message_handlers));
}

void listener_start_push_to_list(void) {
	//TODO LIST NOT IMPLEMENTED YET
	memset(&(af3_v2v.message_buffers), 0, sizeof(af3_v2v.message_buffers));
	v2v_msg_handler_setup(&(af3_v2v.message_buffers), &(af3_v2v.rx_buffer_mutex)
			);
	af3_v2v.message_handlers.platoon_handler =
			v2v_msg_handler_store_platoon;
	af3_v2v.message_handlers.leader_heartbeat_handler =
			v2v_msg_handler_store_leader_heartbeat;
	af3_v2v.message_handlers.follower_handler =
			v2v_msg_handler_store_follower;
	af3_v2v.message_handlers.leave_platoon_handler =
			v2v_msg_handler_store_leave_platoon;
	af3_v2v.message_handlers.split_handler =
			v2v_msg_handler_store_split;
	af3_v2v.message_handlers.new_leader_handler =
			v2v_msg_handler_store_new_leader;
	af3_v2v.message_handlers.fuse_platoon_handler =
			v2v_msg_handler_store_fuse_platoon;
	af3_v2v.message_handlers.handshake_handler =
			v2v_msg_handler_store_handshake;
	af3_v2v_listen(&(af3_v2v.message_handlers));
}

//END SECTION UNDER CONSTRUCTION

int af3_v2v_activate(uint16_t port, GEN_TYPE_int local_uid, bool start_listener)
{
	//setup local dev
	af3_v2v.local_addr_n_port.sin_family = AF_INET;
	af3_v2v.local_addr_n_port.sin_addr.s_addr = htonl(INADDR_ANY);
	af3_v2v.local_addr_n_port.sin_port = htons(port);
	//setup target for tx
	af3_v2v.remote_addr_n_port_tx.sin_family = AF_INET;
	af3_v2v.remote_addr_n_port_tx.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	af3_v2v.remote_addr_n_port_tx.sin_port = htons(port);
	//setup target for rx
	af3_v2v.remote_addr_n_port_rx.sin_family = AF_INET;
	af3_v2v.remote_addr_n_port_rx.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	af3_v2v.remote_addr_n_port_rx.sin_port = htons(INADDR_ANY);
	//initialize broadcast socket
	af3_v2v.bc_socket_fd = v2v_comm_init(&af3_v2v.local_addr_n_port);
	//initialize actual_rx_addr_len to indicate buffer size to recvfrom -
	//otherwise IP after first recvfrom won't be correct. See: man recvfrom
	af3_v2v.actual_rx_addr_len = sizeof(in_addr_t) * 8;
	//allow later created listener thread to run
	af3_v2v.listener_thread_run = 0x01;
	//initialize mutex used for tramsitter callbacks
	pthread_mutex_init(&(af3_v2v.tx_mutex), NULL);
	//set local uid
	af3_v2v.local_uid = local_uid;
	/*Init leader and preceding car UID to -1 to disable filtering until actual
	 *UIDs were set. */
	af3_v2v.preceding_vehicle_of_interest_uid = -1;
	af3_v2v.leader_vehicle_of_interest_uid = -1;
	//initialize mutex used for accessing unique ids
	pthread_mutex_init(&(af3_v2v.uid_mutex),NULL);
	//initialize mutex used for accessing the receive buffer
	pthread_mutex_init(&(af3_v2v.rx_buffer_mutex),NULL);
	//start the listener thread if desired
	if(start_listener) {
		listener_start_push_to_list();
	}
	return af3_v2v.bc_socket_fd;
}

int af3_v2v_deactivate(void) {
	af3_v2v.listener_thread_run = 0;
	//send cancellation request to listener thread
	pthread_cancel(af3_v2v.listener_thread_id);
	//wait until the cancellation request was accepted
	pthread_join(af3_v2v.listener_thread_id, NULL);
	//now destroy the now unused mutexes
	pthread_mutex_destroy(&(af3_v2v.tx_mutex));
	pthread_mutex_destroy(&(af3_v2v.uid_mutex));
	pthread_mutex_destroy(&(af3_v2v.rx_buffer_mutex));
	return(v2v_comm_exit(af3_v2v.bc_socket_fd));
}

void af3_v2v_set_leader_vehicle_of_interest_uid(GEN_TYPE_int uid,
		GEN_TYPE_boolean uid_noval) {
	pthread_mutex_lock(&(af3_v2v.uid_mutex));
	if(uid_noval != true) {
		af3_v2v.leader_vehicle_of_interest_uid = uid;
	}
	else {
		af3_v2v.leader_vehicle_of_interest_uid = -1;
		//negaive numbers here will disable uid filtering
	}
	pthread_mutex_unlock(&(af3_v2v.uid_mutex));
}

void af3_v2v_set_preceding_vehicle_of_interest_uid(GEN_TYPE_int uid,
		GEN_TYPE_boolean uid_noval) {
	pthread_mutex_lock(&(af3_v2v.uid_mutex));
	if(uid_noval != true) {
		af3_v2v.preceding_vehicle_of_interest_uid = uid;
	}
	else {
		af3_v2v.preceding_vehicle_of_interest_uid = -1;
		//negaive numbers here will disable uid filtering
	}
	pthread_mutex_unlock(&(af3_v2v.uid_mutex));
}

ssize_t af3_v2v_send_msg(uint8_t msg_type, uint8_t* p_msg, size_t msg_len) {
	/*construct a message form the 1 Byte message identifier and the actual
	 *message and transmit it as a udp packet. */

	uint8_t tx_buffer[V2V_MAX_UDP_PAYLOAD_BYTES] = {0};
	size_t tx_len = 0;
	af3_v2v_generic_message_t* af3_v2v_message =
			(af3_v2v_generic_message_t*) tx_buffer;
	ssize_t res = -1;

	//write message type identifier and uid to tx buffer
	af3_v2v_message->type_id = msg_type;
	tx_len += sizeof(af3_v2v_message->type_id);
	af3_v2v_message->source_uid = af3_v2v.local_uid;
	tx_len += sizeof(af3_v2v_message->source_uid);
	af3_v2v_message->payload_length = msg_len;
	tx_len += sizeof(af3_v2v_message->payload_length);

	//TODO avoid writing too many bytes to the buffer
	memcpy(&(af3_v2v_message->actual_message_start), p_msg, msg_len);
	tx_len += msg_len;

	//avoid sending too large messages
	if(tx_len > V2V_MAX_UDP_PAYLOAD_BYTES) {
		tx_len = V2V_MAX_UDP_PAYLOAD_BYTES;
		debug_print(DEBUG_PRINT_LEVEL_FEW, "V2V: Message payload too large."
				"Transmitting concatenated message.\n");
	}
	//transmit
	/* as this callback is intended to be called form external we do not know
	 * how many threads will try to use it therefore lock function call
	 * containing sendto() call to avoid data loss. */
	pthread_mutex_lock(&(af3_v2v.tx_mutex));
	res = v2v_comm_tx_raw(tx_buffer, tx_len, &(af3_v2v.remote_addr_n_port_tx),
			af3_v2v.bc_socket_fd);
	pthread_mutex_unlock(&(af3_v2v.tx_mutex));
	return res;
}

void af3_v2v_get_platoon(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.platoon_buffer.is_new){
		memcpy(p_target_struct, af3_v2v.message_buffers.platoon_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.platoon_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}
void af3_v2v_get_leader_heartbeat(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.leader_heartbeat_buffer.is_new){
		memcpy(p_target_struct,
				af3_v2v.message_buffers.leader_heartbeat_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.leader_heartbeat_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}
void af3_v2v_get_follower(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.follower_buffer.is_new){
		memcpy(p_target_struct, af3_v2v.message_buffers.follower_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.follower_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}
void af3_v2v_get_leave_platoon(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.leave_platoon_buffer.is_new){
		memcpy(p_target_struct,
				af3_v2v.message_buffers.leave_platoon_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.leave_platoon_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}
void af3_v2v_get_split(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.split_buffer.is_new){
		memcpy(p_target_struct, af3_v2v.message_buffers.split_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.split_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}
void af3_v2v_get_new_leader(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.new_leader_buffer.is_new){
		memcpy(p_target_struct,
				af3_v2v.message_buffers.new_leader_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.new_leader_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}
void af3_v2v_get_fuse_platoon(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.fuse_platoon_buffer.is_new){
		memcpy(p_target_struct,
				af3_v2v.message_buffers.fuse_platoon_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.fuse_platoon_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}
void af3_v2v_get_handshake(uint8_t* p_target_struct,
		GEN_TYPE_boolean* p_variable_to_write_noval, size_t target_size) {
	pthread_mutex_lock(&(af3_v2v.rx_buffer_mutex));
	if(af3_v2v.message_buffers.handshake_buffer.is_new){
		memcpy(p_target_struct,
				af3_v2v.message_buffers.handshake_buffer.message,
				target_size); //TODO check size
		*p_variable_to_write_noval = false;
		af3_v2v.message_buffers.handshake_buffer.is_new = 0x0;
	}
	else {
		*p_variable_to_write_noval = true;
	}
	pthread_mutex_unlock(&(af3_v2v.rx_buffer_mutex));
}

void af3_v2v_send_platoon(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_PLATOON, p_src_struct, src_size);
}
void af3_v2v_send_leader_heartbeat(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_LEADER_HEARTBEAT, p_src_struct, src_size);
}
void af3_v2v_send_follower(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_FOLLOWER, p_src_struct, src_size);
}
void af3_v2v_send_leave_platoon(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_LEAVE_PLATOON, p_src_struct, src_size);
}
void af3_v2v_send_split(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_SPLIT, p_src_struct, src_size);
}
void af3_v2v_send_new_leader(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_NEW_LEADER, p_src_struct, src_size);
}
void af3_v2v_send_fuse_platoon(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_FUSE_PLATOON, p_src_struct, src_size);
}
void af3_v2v_send_handshake(uint8_t* p_src_struct, size_t src_size) {
	af3_v2v_send_msg(V2V_MSG_TYPE_HANDSHAKE, p_src_struct, src_size);
}

/**
 * \brief Extract message type and source uid to given variables.
 * Extracts the type id and the uid of the message source from an arbitrary af3
 * v2v message and points given pointers to them.
 * @param p_message_start A pointer pointing at the first byte of the received
 * Message.
 * @param p_msg_type_id A pointer to a pointer that will be pointed to where
 * the type id of the analyzed message is located.
 * @param p_msg_src_uid A pointer to a pointer that will be pointed to where the
 * source uid of the analyzed message is located.
 * @return The number of Bytes processed.
 */
size_t get_msg_type_n_source_n_len(uint8_t* p_message_start,
		uint8_t** p_p_msg_type_id,
		GEN_TYPE_int** p_p_msg_src_uid, size_t** p_p_msg_len) {
	af3_v2v_generic_message_t* p_generic_v2v_msg =
			(af3_v2v_generic_message_t*) p_message_start;
	*p_p_msg_type_id = &(p_generic_v2v_msg->type_id);
	*p_p_msg_src_uid = &(p_generic_v2v_msg->source_uid);
	*p_p_msg_len = &(p_generic_v2v_msg->payload_length);
	debug_print(DEBUG_PRINT_LEVEL_ALL, "V2V: found type: %0X\nsrc: %d\nlen: %zu"
			"\n", **p_p_msg_type_id, **p_p_msg_src_uid, **p_p_msg_len);
	return(sizeof(p_generic_v2v_msg->type_id) +
			sizeof(p_generic_v2v_msg->source_uid) +
			sizeof(p_generic_v2v_msg->payload_length));
}

static inline uint8_t is_it_me_talking(uint16_t message_source) {
	uint8_t res = 0; //default to false
	pthread_mutex_lock(&(af3_v2v.uid_mutex));
	res = (message_source == af3_v2v.local_uid);
	pthread_mutex_unlock(&(af3_v2v.uid_mutex));
	return res;
}

static inline uint8_t is_it_preceding_vehicle_talking(uint16_t message_source) {
	uint8_t res = 1; //default to true
	pthread_mutex_lock(&(af3_v2v.uid_mutex));
	//if preceeding uid is unset return true to allow processing message
	if(af3_v2v.leader_vehicle_of_interest_uid >= 0) {
		res = (message_source == af3_v2v.preceding_vehicle_of_interest_uid);
	}
	pthread_mutex_unlock(&(af3_v2v.uid_mutex));
	return res;
}

static inline uint8_t is_it_leader_vehicle_talking(uint16_t message_source){
	uint8_t res = 1; //default to true
	pthread_mutex_lock(&(af3_v2v.uid_mutex));
	//if leader uid is unset return true to allow processing message
	if(af3_v2v.leader_vehicle_of_interest_uid >= 0) {
		res = (message_source == af3_v2v.leader_vehicle_of_interest_uid);
	}
	pthread_mutex_unlock(&(af3_v2v.uid_mutex));
	return res;
}

void* _af3_v2v_receive_msgs_thread_func(void* p_message_handlers) {
	af3_v2v_handler_functions_container_t* p_message_handlers_my =
			(af3_v2v_handler_functions_container_t*) p_message_handlers;
	size_t read_cnt = 0;
	size_t write_cnt = 0;
	const size_t buffer_size = AF3_V2V_RX_BUFFER_MSG_CAPACITY
			* V2V_MAX_UDP_PAYLOAD_BYTES; /*!rx buffer size for at worst case the
											* given number of messages*/
	uint8_t rx_buffer[buffer_size];
	ssize_t res = -1;
	uint8_t flag_to_few_bytes = 0x00;
	uint8_t* p_msg_type_id = NULL;
	GEN_TYPE_int* p_msg_src_uid = NULL;
	size_t* p_msg_len = NULL;

	while(af3_v2v.listener_thread_run != 0) {
		//buffer_size - write_cnt - 1 yields the remaining buffer capacity
		res = v2v_comm_rx_raw((void*)&(rx_buffer[write_cnt]),
				(buffer_size - write_cnt - 1), &(af3_v2v.remote_addr_n_port_rx),
				&(af3_v2v.actual_rx_addr_len), af3_v2v.bc_socket_fd);
		//handle read errors - just wait and retry
		if(res < 1) {
			debug_print(DEBUG_PRINT_LEVEL_FEW, "V2V: Receiving message(s) "
					"failed. Retrying in 1 second.\n");
			sleep(1);
			continue;
		}
		flag_to_few_bytes = 0x00; //reset read shortage flag
		write_cnt += (uint16_t)res; //increment write counter
		//process buffer content
		while((read_cnt < write_cnt) && flag_to_few_bytes == 0x00) {
			//check whether minimal allowed message size was received
			if((write_cnt - read_cnt) < sizeof(af3_v2v_generic_message_t)) {
				flag_to_few_bytes = 0x01;
				break;
			}
			read_cnt += get_msg_type_n_source_n_len(rx_buffer+read_cnt,
					&p_msg_type_id,	&p_msg_src_uid, &p_msg_len);
			//ignore messages sent by local device
			if(is_it_me_talking(*p_msg_src_uid) != 0) {
				debug_print(DEBUG_PRINT_LEVEL_MANY,"Listening to myself.\n"
						"Not processing content of own V2V Message.\n");
				//advance read counter
				read_cnt += *p_msg_len;
				continue;
			}
			switch(*p_msg_type_id) {
			case V2V_MSG_TYPE_PLATOON:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->platoon_handler != NULL) {
						p_message_handlers_my->platoon_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler "
								"for msg type PLATOON specified.\n Ignoring "
								"received PLATOON message.\n");
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			case V2V_MSG_TYPE_LEADER_HEARTBEAT:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					/*leader heartbeat needs to be filtered to ensure listening
					 *to the leader of the vehicle's own platoon only */
					if(is_it_leader_vehicle_talking(*p_msg_src_uid) < 1) {
						debug_print(DEBUG_PRINT_LEVEL_FEW, "Received V2V "
								"LEADER HEARTBEAT message sent by vehicle with "
								"UID other than leader vehicle.\nIgnroing.\n");
						break;
					}
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->leader_heartbeat_handler != NULL)
					{
						p_message_handlers_my->leader_heartbeat_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler "
								"for msg type LEADER HEARTBEAT specified.\n"
								"Ignoring received LEADER HEARTBEAT message.\n"
								);
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			case V2V_MSG_TYPE_FOLLOWER:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					/*follower needs to be filtered to ensure listening to the
					 * vehicle directly preceding only */
					if(is_it_preceding_vehicle_talking(*p_msg_src_uid) < 1) {
						debug_print(DEBUG_PRINT_LEVEL_FEW, "Received V2V "
								"FOLLOWER message sent by vehicle with UID "
								"other than preceding vehicle.\nIgnroing.\n");
						break;
					}
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->follower_handler != NULL) {
						p_message_handlers_my->follower_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler "
								"for msg type FOLLOWER specified.\n"
								"Ignoring received FOLLOWER message.\n");
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			case V2V_MSG_TYPE_LEAVE_PLATOON:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->leave_platoon_handler != NULL) {
						p_message_handlers_my->leave_platoon_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler "
								"for msg type LEAVE PLATOON specified.\n"
								"Ignoring received PLATOON message.\n");
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			case V2V_MSG_TYPE_SPLIT:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->split_handler != NULL) {
						p_message_handlers_my->split_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler "
								"for msg type SPLIT specified.\n Ignoring "
								"received SPLIT message.\n");
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			case V2V_MSG_TYPE_NEW_LEADER:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->new_leader_handler != NULL) {
						p_message_handlers_my->new_leader_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler"
								"for msg type NEW LEADER specified\n. Ignoring "
								"received NEW LEADER message\n.");
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			case V2V_MSG_TYPE_FUSE_PLATOON:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->fuse_platoon_handler != NULL) {
						p_message_handlers_my->fuse_platoon_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler "
								"for msg type FUSE PLATOON specified\n. "
								"Ignoring received FUSE PLATOON message.\n");
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			case V2V_MSG_TYPE_HANDSHAKE:
				//if to few bytes were received just skip processing and re-read
				if(write_cnt - read_cnt >= *p_msg_len) { //enough bytes read
					//call handler here - handler should check for NULL
					if(p_message_handlers_my->handshake_handler != NULL) {
						p_message_handlers_my->handshake_handler(
								rx_buffer+read_cnt, *p_msg_len);
					}
					else {
						debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: No handler "
								"for msg type HANDSHAKE specified\n. Ignoring "
								"received HANDSHAKE message.\n");
					}
				}
				else {
					flag_to_few_bytes = 0x01;
				}
				break;
			default:
				/*unknown identifier - probably reading at the wrong place -
				 * reset buffer*/
				debug_print(DEBUG_PRINT_LEVEL_FEW, "V2V: Unknown message type "
						"received or read buffer corrupted. Resetting buffer "
						"and counters. PROBABLY SOME MESSAGES WERE LOST.\n");
				write_cnt = 0;
				read_cnt = 0;
				break;
			}
			if(flag_to_few_bytes == 0x00) {
				//advance read counter
				read_cnt += *p_msg_len;
			}
		}
		//reset write and read counter if buffer content was fully handled
		if( write_cnt == read_cnt) {
			write_cnt = 0;
			read_cnt = 0;
		}
	}
	return EXIT_SUCCESS;
}

void af3_v2v_listen(af3_v2v_handler_functions_container_t* p_message_handlers) {
	int res = -1;
	//ensure that listener thread my really run
	af3_v2v.listener_thread_run = 0x01;
	res = pthread_create(&(af3_v2v.listener_thread_id), NULL,
			_af3_v2v_receive_msgs_thread_func, (void*) p_message_handlers);
	if(res != 0) {
		v2v_print_error_msg("creating listener thread.\nWon't receive messages."
				);
	}
}
