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
#ifndef __CANSOCKET_H
#define __CANSOCKET_H

#include <stdbool.h>
#include <stddef.h>
#include <unistd.h>

#include <sys/socket.h>

#include <linux/if.h>
#include <linux/can.h>

#define LIBCANSOCKET_ERROR -1

/** The CAN socket structure. */
struct can_socket {
    int number;						// socket number
    struct sockaddr_can address;	// socket address
    struct ifreq ifreq;				// socket ioctl interface
};
typedef struct can_socket can_socket_t;

typedef struct can_frame can_frame_t;

/**
 * Opens a can socket on the interface (usually "can0") and returns the
 * initialized socket structure.
 */
can_socket_t* can_socket_open(const char *if_name);

/** Closes the socket and frees the socket structure pointer. */
void can_socket_close(can_socket_t* can_socket);

/** Sends a can frame via the socket. */
size_t can_socket_send(can_socket_t* can_socket, can_frame_t* frame);

/** Checks if some bytes are available on the can. */
bool can_socket_available(can_socket_t* can_socket);

/** Receives a can frame via the socket. */
size_t can_socket_receive(can_socket_t* can_socket, can_frame_t* frame);

#endif // __CANSOCKET_H
