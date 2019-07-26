#ifndef CONNECTION_H
#define CONNECTION_H

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <mutex>
#include <vector>

class Connection {
public:
	int sock;
	struct sockaddr_in addr;

	Connection(int sock, struct sockaddr_in addr);
};

extern std::mutex connection_list_mutex;
extern std::vector<Connection> connection_list;

#endif
