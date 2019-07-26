/*
 * v2v_udp_basic.c
 *
 *  Created on: 11.05.2018
 *      Author: Constantin
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "v2v_udp_basic.h"
#include "debugprint.h"

void v2v_print_error_msg(char* p_text){
	debug_print(DEBUG_PRINT_LEVEL_FEW,"Error %s. Errorcode: %s.\n",p_text,
				strerror(errno));
}

int v2v_comm_init(struct sockaddr_in* p_local_sockaddr_in) {
	int res = -1;
	int socket_fd = -1;
	int broadcast_enable = 1;

	//create socket
	socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socket_fd < 1) {
		v2v_print_error_msg("creating socket. Exiting.");
		return socket_fd;
	}

	//set socket to allow broadcasts
	res = setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable,
			sizeof(broadcast_enable));
	if(res < 0) {
		v2v_print_error_msg("setting socket to broadcast mode. Exiting.");
		return res;
	}

	//bind socket
	res = bind(socket_fd, (struct sockaddr*) p_local_sockaddr_in,
			sizeof(*p_local_sockaddr_in));
	if(res < 0) {
		v2v_print_error_msg("using bind(). Exiting.");
		return res;
	}
	return socket_fd;
}

int v2v_comm_exit(int socket_fd_to_exit) {
	int res = -1;

	//close socket
	res = close(socket_fd_to_exit);
	if(res < 0) {
		v2v_print_error_msg("closing socket. Exiting.");
	}
	return res;
}

ssize_t v2v_comm_tx_raw(void* tx_buffer, size_t tx_len, struct sockaddr_in*
		p_remote_sockaddr_in, int socket_fd) {
	ssize_t res = -1;

	debug_print(DEBUG_PRINT_LEVEL_MANY, "V2V: Sending %zu bytes UDP payload.\n",
			tx_len);
	res = sendto(socket_fd,tx_buffer,tx_len, 0,
			(struct sockaddr*) p_remote_sockaddr_in,
			sizeof(*p_remote_sockaddr_in));
	if(res < 0) {
			v2v_print_error_msg("sendto() failed. Skipping\n");
	}
	return res;
}

ssize_t v2v_comm_rx_raw(void* rx_buffer, size_t rx_buffer_size,
		struct sockaddr_in* p_remote_sockaddr_in,
		socklen_t* p_remote_sockaddr_in_len, int socket_fd) {
	ssize_t res = -1;

	res = recvfrom(socket_fd, rx_buffer, rx_buffer_size, 0 ,
			(struct sockaddr*) p_remote_sockaddr_in,p_remote_sockaddr_in_len);
	if(res < 0) {
			v2v_print_error_msg("recvfrom() failed. Skipping\n");
	}
	debug_print(DEBUG_PRINT_LEVEL_MANY,"V2V: Received %zu bytes raw data.\n",
			res);
	return res;
}
