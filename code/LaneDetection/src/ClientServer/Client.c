#include <pthread.h>
#include <signal.h>
#include <netdb.h>
#include <sys/types.h>
#include <string.h>
#include <sys/time.h>
#include "Defs.h"
#include "Data.h"
#include <errno.h>

pthread_mutex_t values_mutex;
LDValues values;
struct timeval tval_lastUpdate;
bool connected = false;
char server_hostname[100];
char server_port[100];

size_t read_all(int fd, void *buf, size_t count) {
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

void* receiver(void* willBeNull) {
	(void)willBeNull;//suppress compiler warning
	LDValues vals;
	memset(&vals, 0, sizeof(LDValues));
	int sock;
	pthread_mutex_init(&values_mutex, NULL);

	//forver: 1. try to connect. 2. read data. 3. connection broken -> back to 1
	while (true) {

		while (true) { //endless loop: trying to reach server
			if (client_connect(server_hostname, server_port, &sock)) {
				connected = true;
				break;
			} else {
				sleep(1);
				printf("LaneDetection: constantly trying to reconnect\n");
			}
		}

		printf("LaneDetectio: Connected -> Reading data\n");
		while (true) { //endless loop: read data
			int expectedSize = sizeof(LDValues);
			int read_bytes = read_all(sock, &vals, expectedSize);
			if (errno == ECONNRESET || read_bytes != expectedSize) {
				connected = false;
				printf("LaneDetection: server gone :-(");
				close(sock);
				sleep(1);
				break;
			}
			// Copy into values
			CHECK(0 == pthread_mutex_lock(&values_mutex));
			memcpy(&values, &vals, sizeof(LDValues));
			CHECK(0 == pthread_mutex_unlock(&values_mutex));
			//printf("Received: ");
			//printhex((char*) &vals, sizeof(Values));
			gettimeofday(&tval_lastUpdate, NULL);

		}
	}
	//exit(-1);
	return NULL ;
}

void init_client(const char *hostname, const char *port) {
	strcpy(server_hostname, hostname);
	strcpy(server_port, port);
	pthread_t receive_thread;
	CHECK(0 <= pthread_create(&receive_thread, NULL, &receiver,NULL));
}

AF3LDValues get_values() {
	AF3LDValues vals;
	CHECK(0 == pthread_mutex_lock(&values_mutex));
	memcpy(&vals, &values, sizeof(LDValues));
	CHECK(0 == pthread_mutex_unlock(&values_mutex));
	struct timeval tval_now;
	gettimeofday(&tval_now, NULL);
	unsigned long usec_diff = (tval_now.tv_sec - tval_lastUpdate.tv_sec)
			* 1000000 + (tval_now.tv_usec - tval_lastUpdate.tv_usec);
	vals.msSinceLastUpdate = usec_diff / 1000;
	vals.ldConnected = connected;
	return vals;
}

#ifdef TEST
int main() {
	init_client("127.0.0.1", "4444");
	AF3LDValues vals;

	while(1) {
		vals = get_values();
		printf("%s \t %s \t %2.7f \t %2.7f \t %2.7f \t %lu %s \n",
				vals.detectedLeft ? "true" : "false",
				vals.detectedRight ? "true" : "false",
				vals.yawAngle,
				vals.distanceLeft,
				vals.distanceRight,
				vals.msSinceLastUpdate,
				vals.ldConnected ? "true" : "false");
		usleep(100000);
	}
}
#endif
