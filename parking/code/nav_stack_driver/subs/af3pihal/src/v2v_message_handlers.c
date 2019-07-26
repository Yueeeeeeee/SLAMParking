/*
 * v2v_message_handlers.c
 *
 *  Created on: 13.05.2018
 *      Author: Constantin
 */


#include "v2v_message_handlers.h"
#include "v2v_message_types.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include "af3_v2v.h"
#include "debugprint.h"
#include <string.h>

struct {
	v2v_msg_handler_buffers_t* p_buffers;
	pthread_mutex_t* p_mutex;
}v2v_message_handlers;

void v2v_msg_handler_debug_monitor_unknown(uint8_t* p_msg, size_t msg_len) {
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"UNKNOWN TYPE\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_debug_monitor_platoon(uint8_t* p_msg, size_t msg_len) {
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"PLATOON\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_debug_monitor_leader_heartbeat(uint8_t* p_msg,
		size_t msg_len) {
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"LEADER HEARTBEAT\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_debug_monitor_follower(uint8_t* p_msg, size_t msg_len) {
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"FOLLOWER\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_debug_monitor_leave_platoon(uint8_t* p_msg, size_t msg_len)
{
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"LEAVE PLATOON\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_debug_monitor_split(uint8_t* p_msg, size_t msg_len) {
	return;
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"SPLIT\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_debug_monitor_new_leader(uint8_t* p_msg, size_t msg_len) {
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"NEW LEADER\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_debug_monitor_fuse_platoon(uint8_t* p_msg, size_t msg_len)
{
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"FUSE PLATOON\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}

void v2v_msg_handler_debug_monitor_handshake(uint8_t* p_msg, size_t msg_len)
{
	//print timestamp first
	time_t my_timestamp = time(NULL);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "%s",ctime(&my_timestamp));

	if(p_msg == NULL) {
		debug_print(DEBUG_PRINT_LEVEL_FEW,"received NULL pointer - not trying"
				" to process message\n"
				);
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Received v2v message of type: ");
	/*print msg type in bold - see:
	 * https://www.unix.com/programming/21073-bold-text.html (13.05.18)*/
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[1m",27); //bold on
	debug_print(DEBUG_PRINT_LEVEL_FEW,"HANDSHAKE\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW,"%c[0m",27); //bold off
	debug_print(DEBUG_PRINT_LEVEL_FEW,"----\n");

	debug_print(DEBUG_PRINT_LEVEL_FEW, "Msg payload length: %zu\n", msg_len);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Payload: ");
	for(size_t i = 0; i < msg_len; i++) {
		debug_print(DEBUG_PRINT_LEVEL_FEW, "%0X", p_msg[i]);
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "\n");
}
void v2v_msg_handler_setup(v2v_msg_handler_buffers_t* p_buffers,
		pthread_mutex_t* p_mutex) {
	v2v_message_handlers.p_buffers = p_buffers;
	v2v_message_handlers.p_mutex = p_mutex;
}

void v2v_msg_handler_store_platoon(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_platoon(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->platoon_buffer.message, p_msg,
			msg_len);
	v2v_message_handlers.p_buffers->platoon_buffer.actual_message_length =
			msg_len;
	v2v_message_handlers.p_buffers->platoon_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
void v2v_msg_handler_store_leader_heartbeat(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_leader_heartbeat(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->leader_heartbeat_buffer.message,
			p_msg, msg_len);
	v2v_message_handlers.p_buffers->
	leader_heartbeat_buffer.actual_message_length = msg_len;
	v2v_message_handlers.p_buffers->leader_heartbeat_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
void v2v_msg_handler_store_follower(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_follower(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->follower_buffer.message, p_msg,
			msg_len);
	v2v_message_handlers.p_buffers->follower_buffer.actual_message_length =
			msg_len;
	v2v_message_handlers.p_buffers->follower_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
void v2v_msg_handler_store_leave_platoon(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_leave_platoon(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->leave_platoon_buffer.message, p_msg,
			msg_len);
	v2v_message_handlers.p_buffers->leave_platoon_buffer.actual_message_length =
			msg_len;
	v2v_message_handlers.p_buffers->leave_platoon_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
void v2v_msg_handler_store_split(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_split(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->split_buffer.message, p_msg, msg_len)
			;
	v2v_message_handlers.p_buffers->split_buffer.actual_message_length = msg_len
			;
	v2v_message_handlers.p_buffers->split_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
void v2v_msg_handler_store_new_leader(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_new_leader(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->new_leader_buffer.message, p_msg,
			msg_len);
	v2v_message_handlers.p_buffers->new_leader_buffer.actual_message_length =
			msg_len;
	v2v_message_handlers.p_buffers->new_leader_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
void v2v_msg_handler_store_fuse_platoon(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_fuse_platoon(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->fuse_platoon_buffer.message, p_msg,
			msg_len);
	v2v_message_handlers.p_buffers->fuse_platoon_buffer.actual_message_length =
			msg_len;
	v2v_message_handlers.p_buffers->fuse_platoon_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
void v2v_msg_handler_store_handshake(uint8_t* p_msg, size_t msg_len) {
	v2v_msg_handler_debug_monitor_handshake(p_msg, msg_len);
	pthread_mutex_lock(v2v_message_handlers.p_mutex);
	memcpy(v2v_message_handlers.p_buffers->handshake_buffer.message, p_msg,
			msg_len);
	v2v_message_handlers.p_buffers->handshake_buffer.actual_message_length =
			msg_len;
	v2v_message_handlers.p_buffers->handshake_buffer.is_new = 0x01;
	pthread_mutex_unlock(v2v_message_handlers.p_mutex);
}
