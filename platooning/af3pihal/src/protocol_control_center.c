/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *    Constantin Dresel - added receive function
 *******************************************************************************/
#include "protocol_control_center.h"

#include <arpa/inet.h>
#include <debugprint.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int cc_socket = 0;
static struct sockaddr_in cc_server;

void protocol_control_center_create(protocol_control_center_configuration_t* config) {
	cc_socket = socket(AF_INET, SOCK_STREAM, 0);
	if(cc_socket < 0) {
		perror("Error creating socket to control center.");
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Control center socket created.\n");

	cc_server.sin_addr.s_addr = inet_addr(config->server_ip_address);
	cc_server.sin_family = AF_INET;
	cc_server.sin_port = htons(config->server_port);

	if(connect(cc_socket, (struct sockaddr *)&cc_server, sizeof(cc_server)) < 0) {
		perror("Connection to control center failed.");
		close(cc_socket);
		cc_socket = 0;
		return;
	}
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Control center connection established.\n");
}

void protocol_control_center_write(char* text, size_t len) {
	if(cc_socket > 0) {
		if(send(cc_socket, text, len, 0) < 0) {
			perror("Sending to control center failed.");
		}
	} else {
		perror("Socket to control center not open.");
	}
}
// receive from socket to buffer
ssize_t protocol_control_center_receive(char* text, size_t len){
	//debug_print(DEBUG_PRINT_LEVEL_FEW, "protocol_control_center_receive \n");
	ssize_t res = -1; //default to error return value
	if(cc_socket > 0) {
		res = recv(cc_socket, text, len, 0);
		if(res < 0) {
			perror("Receiving from control center failed.");
		}
	}
	else {
		res = -1;
		perror("Socket to control center not open.");
	}
	return res;
}

void protocol_control_center_terminate() {
	if(cc_socket > 0) {
		close(cc_socket);
	}
}
