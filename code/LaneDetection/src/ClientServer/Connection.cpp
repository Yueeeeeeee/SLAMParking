#include "Defs.h"
#include "Connection.hpp"

std::mutex connection_list_mutex;
std::vector<Connection> connection_list;

Connection::Connection(int sock, struct sockaddr_in addr) {
	this->sock = sock;
	this->addr = addr;
}
