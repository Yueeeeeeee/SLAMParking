/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Sebastian Neubauer, Florian Hoelzl - initial API and implementation
 *******************************************************************************/

#include <camera_client.h>
#include <timeutil.h>
#include <pthread.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <errno.h>

typedef struct {
	float distanceLeft;
	float distanceRight;
	float yawAngle;
	bool detectedLeft;
	bool detectedRight;
	int precedingRoverDistance; //mm
	int precedingRoverId;
	bool precedingRoverRecognized;
	bool curveDetected;
	float curveRadius;
} camera_values_t;

#define CHECK(B) do { if (!(B)) { \
	perror("Error occured."); \
	exit(1); \
	}} while (0)

#define ESUCCESS 0

static pthread_mutex_t values_mutex;
static camera_values_t values;
static bool camera_connected;
static uint64_t last_update_time;
static char server_hostname[100];
static char server_port[100];

// server connection
bool camera_client_is_connected() {
	bool result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = camera_connected;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

uint64_t camera_client_get_ms_since_last_update() {
	uint64_t result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = time_util_get_elapsed_micros_since(last_update_time) / MILLIS_IN_MICROS;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

// lane detection
float camera_client_get_distance_left() {
	float result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.distanceLeft;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

float camera_client_get_distance_right() {
	float result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.distanceRight;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

float camera_client_get_yaw_angle() {
	float result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.yawAngle;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

bool camera_client_get_detection_state_left() {
	bool result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.detectedLeft;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

bool camera_client_get_detection_state_right() {
	bool result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.detectedRight;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

// rover detection
bool camera_client_is_preceding_rover_recognized() {
	bool result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.precedingRoverRecognized;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}
int camera_client_get_preceding_rover_distance() {
	int result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.precedingRoverDistance;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}
int camera_client_get_preceding_rover_id() {
	int result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.precedingRoverId;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

// curve detection
bool camera_client_is_curve_detected() {
	bool result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.curveDetected;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

float camera_client_get_curve_radius() {
	float result;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	result = values.curveRadius;
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	return result;
}

// network access
static size_t read_all(int fd, void *buf, size_t count) {
	size_t res = 0;
	while (count > 0) {
		ssize_t ret = read(fd, buf + res, count);
		if (ret <= 0) {
			return ret;
		}
		count -= ret;
		res += ret;
	}
	return res;
}

bool client_connect(const char *host, const char *port, int *out_sock) {
	bool success = false;
	struct addrinfo hints;
	struct addrinfo *result, *rp;
	int sfd;

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_UNSPEC; /* Allow IPv4 or IPv6 */
	hints.ai_flags = 0;
	hints.ai_protocol = IPPROTO_TCP; /* Explicitly specify tcp. getaddrinfo should deliver only one result */

	CHECK(0 == getaddrinfo(host, port, &hints, &result));
	for (rp = result; rp != NULL ; rp = rp->ai_next) {
		sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (sfd == -1) {
			//skip
			continue;
		}
		errno=0;
		int connectSuccess = connect(sfd, rp->ai_addr, rp->ai_addrlen);
		if (connectSuccess == 0 && errno == ESUCCESS) {
			success = true;
			break;
		} else {
			perror("Attempt to connect unsucessfull because:");
		}
		//if no success close sfd
		close(sfd);
	}

	freeaddrinfo(result);
	*out_sock = sfd;
	return success;
}

static void* receiver() {
	camera_values_t vals;
	int sock = 0;

	memset(&vals, 0, sizeof(camera_values_t));

	while (true) { // endless loop: connect and receive
		while (true) { // endless loop: trying to reach server
			if (client_connect(server_hostname, server_port, &sock)) {
				camera_connected = true;
				break;
			} else {
				camera_connected = false;
				sleep(1);
				debug_print(DEBUG_PRINT_LEVEL_FEW, "LaneDetection: constantly trying to reconnect\n");
			}
		}
		debug_print(DEBUG_PRINT_LEVEL_FEW, "LaneDetection: Connected -> Reading data\n");

		while(true) { // endless loop: read data from server
			int read_bytes = read_all(sock, &vals, sizeof(camera_values_t));
			if (errno == ECONNRESET || read_bytes != sizeof(camera_values_t)) {
				camera_connected = false;
				printf("LaneDetection: server gone :-(");
				close(sock);
				sleep(1);
				break;
			}
			// copy temporary buffer into exchange variable
			CHECK(0 == pthread_mutex_lock(&values_mutex));
			memcpy(&values, &vals, sizeof(camera_values_t));
			last_update_time = time_util_get_current_micros();
			CHECK(0 == pthread_mutex_unlock(&values_mutex));
			debug_print(DEBUG_PRINT_LEVEL_MANY, "%s \t %s \t %2.7f \t %2.7f \t %2.7f \n",
					vals.detectedLeft ? "true" : "false",
					vals.detectedRight ? "true" : "false",
					vals.yawAngle,
					vals.distanceLeft,
					vals.distanceRight);
		}
	}
	return NULL;
}

void camera_client_initialize(const char *hostname, const char *port) {
	pthread_t receive_thread;
	strcpy(server_hostname, hostname);
	strcpy(server_port, port);
	pthread_mutex_init(&values_mutex, NULL);
	last_update_time = time_util_get_current_micros();
	CHECK(0 <= pthread_create(&receive_thread, NULL, &receiver, NULL));
}
