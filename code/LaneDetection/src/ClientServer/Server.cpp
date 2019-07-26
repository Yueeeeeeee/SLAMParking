#include <signal.h>
#include <thread>
#include <cstring>

#include "Defs.h"
#include "Data.h"
#include "Connection.hpp"

ssize_t write_all(int fd, const void *buf, size_t count) {
	size_t res = 0;
	while (count > 0) {
		ssize_t ret = write(fd, buf, count);
		if (ret <= 0) {
			return ret;
		}
		count -= ret;
		res += ret;
	}
	return res;
}

// Send to all connections
void send_all(const void *buf, size_t count) {
	//printf("Send: \n");
	//printhex((char*) buf, count);
	std::lock_guard<std::mutex> lock(connection_list_mutex);
	auto con = connection_list.begin();

	while (con != connection_list.end()) {
		if ((ssize_t) count != write_all(con->sock, buf, count)) {
			// Remove connection on error
			close(con->sock); // Ignore return value
			connection_list.erase(con);
			printf("Connection closed (currently %zu)\n", connection_list.size());
		} else {
			con++;
		}
	}
}

void send_values(const LDValues *values) {
	send_all(values, sizeof(LDValues));
}

#ifdef TEST
void sender_thread();
#endif

void real_init_server() {
	int lsock, sock;
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr = { INADDR_ANY, };
	addr.sin_port = htons(PORT);
	socklen_t addr_len = sizeof(addr);

	// Setup socket
	CHECK(0 <= (lsock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)));
	// This enables fast killing and restarting of server, without error: Address already in use
    int reuse = 1;
    CHECK(0 <= setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)));
    CHECK(0 <= setsockopt(lsock, SOL_SOCKET, SO_REUSEPORT, (const char*)&reuse, sizeof(reuse)));

	CHECK(0 <= bind(lsock, (struct sockaddr*) &addr, sizeof(addr)));
	CHECK(!listen(lsock, 0x10));
	printf("Server is running\n");

	// Ignore sigpipe signals for broken connections (they will be removed on
	// the next write).
	signal(SIGPIPE, SIG_IGN);
	printf("Server is running\n");
#ifdef TEST
	// Start thread
	std::thread(sender_thread).detach();
#endif

	while (true) {
		CHECK(0 <= (sock = accept(lsock, (struct sockaddr*) &addr, &addr_len)));

		// Add connection
		std::lock_guard<std::mutex> lock(connection_list_mutex);
		connection_list.emplace_back(sock, addr);
		printf("Incoming connection (currently %zu)\n", connection_list.size());
	}

	CHECK(0 <= close(lsock));
}

void init_server() {
	std::thread(real_init_server).detach();
}

#ifdef TEST
void sender_thread() {
	printf("Thread is running\n");

	LDValues vals;
	memset(&vals, 0, sizeof(LDValues));
	vals.detectedLeft = true;
	vals.detectedRight = false;
	vals.distanceLeft = 1.0;
	vals.distanceRight = 2.0;
	vals.yawAngle = 3.0;

	while (true) {
		// Send a value each second
		send_all(&vals, sizeof(LDValues));
		usleep(100000);
		vals.distanceRight+=1;
	}
}

int main() {
	init_server();
	sleep(1000);
}
#endif
