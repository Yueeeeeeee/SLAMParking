/** @file v2v_udp_basic.h
 * \brief Abstraction layer for vehicle 2 vehicle communication via UDP.
 *
 * Basic UDP broadcast functionality made more comfortable to use for vehicle 2
 * vehicle communication by wrappers etc..
 * Created on: 11.05.2018
 * Author: Constantin
 */

#ifndef V2V_UDP_BASIC_H_
#define V2V_UDP_BASIC_H_

#include <unistd.h>
#include <netinet/in.h>

/** Define the maximum permitted payload size - see:
 * https://stackoverflow.com/questions/1098897/
 * what-is-the-largest-safe-udp-packet-size-on-the-internet (12.05.2018)
 */
#define V2V_MAX_UDP_PAYLOAD_BYTES 576 //IPv4

/**
 * \brief Prints the given error msg combined with the an error description.
 * Prints an error message containing the given message string and the error
 * description from errno. CAUTION: global_debug_print_level has to be set to
 * DEBUG_PRINT_LEVEL_FEW or something more verbose for this function to produce
 * any output.
 * @param p_text a pointer to the string to print
 */
void v2v_print_error_msg(char* p_text);

/**
 * \brief Initialize socket communication for v2v data exchange.
 * Initializes socket communication for data exchange using udp broadcasts.
 * @param p_local_sockaddr_in pointer to a sockaddr_in struct specifying the
 * specifying the address and port to use for the local device.
 * @return The socket file descriptor on success a negative value otherwise.
 */
int v2v_comm_init(struct sockaddr_in* p_local_sockaddr_in);

/**
 * \brief Closes the socket used for v2v communication.
 * Closes the specified socket specified by the given file descriptor using
 * close().
 * @param socket_fd_to_exit The file descriptor of the socket to close.
 * @return Zero on success, negative otherwise.
 */
int v2v_comm_exit(int socket_fd_to_exit);

/**
 * \brief Broadcast the given arbitrary raw msg via udp.
 * Sends the specified length from the given buffer as payload of a udp
 * broadcast packet.
 * @param tx_buffer The buffer containing the message to transmit.
 * @param tx_len The number of Bytes to transmit from tx_buffer.
 * @param p_remote_sockaddr_in A pointer to a sockaddr_in struct specifying the
 * remote target to send packets to (may be setup to broadcast address).
 * @param socket_fd The file descriptor of the socket to use for transmitting.
 * @return The number of bytes transmitted on success, negative otherwise.
 */
ssize_t v2v_comm_tx_raw(void* tx_buffer, size_t tx_len, struct sockaddr_in*
		p_remote_sockaddr_in, int socket_fd);

/**
 * \brief Receives arbitrary raw udp messages.
 * Receives arbitrary raw udp messages and writes them to the specified
 * rx_buffer.
 * @param rx_buffer The buffer to write received messages to
 * @param rx_buffer_size The capacity of the rx_buffer specified in bytes.
 * @param p_remote_sockaddr_in A pointer to a sockaddr_in struct specifying the
 * remote target to receive packets from (may be setup to receive any message).
 * @param p_remote_sockaddr_in_len A pointer to a socklen_t variable where the
 * length of the remote transmitters address will be stored.
 * @param socket_fd The file descriptor of the socket to use for receiving.
 * @return The number of bytes received on success, 0 if the socket was closed,
 * negative otherwise.
 */
ssize_t v2v_comm_rx_raw(void* rx_buffer, size_t rx_buffer_size,
		struct sockaddr_in* p_remote_sockaddr_in,
		socklen_t* p_remote_sockaddr_in_len, int socket_fd);

#endif /*V2V_UDP_BASIC_H_*/
