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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <bits/time.h>
#include <cansocket.h>

#include <sys/ioctl.h>
#include <sys/select.h>

#include <linux/can/raw.h>

can_socket_t* can_socket_open(const char *if_name) {
	can_socket_t* can_socket = malloc(sizeof(can_socket_t));

	// open socket
	if ((can_socket->number = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("libcansocket:open_socket");
		return NULL;
	}
	int flags = fcntl(can_socket->number, F_GETFL, 0);
	if (flags == -1) {
		perror("Could not get socket flags during can_socket_open().\n");
		return NULL;
	}
	fcntl(can_socket->number, F_SETFL, flags | O_NONBLOCK);

	can_socket->address.can_family = AF_CAN;
	strcpy(can_socket->ifreq.ifr_name, if_name);
	// map if name to interface index -> will fail if interface does not exist
	if (ioctl(can_socket->number, SIOCGIFINDEX, &can_socket->ifreq) < 0) {
		perror("libcansocket:SIOCGIFINDEX");
		return NULL;
	}
	can_socket->address.can_ifindex = can_socket->ifreq.ifr_ifindex;

	// bind socket to interface -> will fail if interface is not up
	if (bind(can_socket->number, (struct sockaddr *) &can_socket->address,
			sizeof(can_socket->address)) < 0) {
		perror("libcansocket:bind");
		return NULL;
	}

	return can_socket;
}

void can_socket_close(can_socket_t* can_socket) {
	close(can_socket->number);
	free(can_socket);
}

size_t can_socket_send(can_socket_t* can_socket, can_frame_t* frame) {
	size_t nbytes;

	/* send frame */
	if ((nbytes = write(can_socket->number, frame, sizeof(*frame)))
			!= sizeof(*frame)) {
		perror("libcansocket:send");
		return LIBCANSOCKET_ERROR;
	}

	return nbytes;
}

bool can_socket_available(can_socket_t* can_socket) {
	struct timeval sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_usec = 500;

	fd_set read_set;
	FD_ZERO(&read_set);
	FD_SET(can_socket->number, &read_set);
	int fdReady = select(can_socket->number + 1, &read_set, NULL, NULL, &sleep_time);
	if (fdReady > 0) {
		return FD_ISSET(can_socket->number, &read_set);
	}
	return false;
}

size_t can_socket_receive(can_socket_t* can_socket, can_frame_t* frame) {
	size_t result = recv(can_socket->number, frame, sizeof(struct can_frame), 0);
	if (result == -1) {
		if (errno == EWOULDBLOCK) {
			return 0;
		}
	}
	if (result != sizeof(*frame)) {
		perror("libcansocket:receive");
		return LIBCANSOCKET_ERROR;
	}
	return result;
}
