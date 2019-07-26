/*
 * VideoSocket.cpp
 *
 *  Created on: Sep 6, 2018
 *      Author: martin
 */

#include "VideoSocket.h"
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>

int VideoSocket::sockfd;
bool VideoSocket::terminationRequest = false;
std::vector<int> VideoSocket::connections;
cv::Mat VideoSocket::currentFrame;

VideoSocket::VideoSocket() {
	sockfd = 0;
	std::thread(setupSocket).detach();
}

VideoSocket::~VideoSocket() {
	terminationRequest = true;
	if (sockfd < 0) {
		std::cout << "Socket closed\n";
		close(sockfd);
	}
}

void VideoSocket::setupSocket() {
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		std::cout << "socket failed\n";
		return;
	}
	struct sockaddr_in address;
	socklen_t address_len = sizeof(address);
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(9090);

	int reuse = 1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
			(const char*) &reuse, sizeof(reuse))) {
		std::cout << "setsockopt failed\n";
		return;
	}

	if (bind(sockfd, (struct sockaddr *) &address, sizeof(address)) < 0) {
		std::cout << "bind failed\n";
		return;
	}

	if (listen(sockfd, 8) < 0) {
		std::cout << "listen failed\n" << errno << "\n";
		return;
	}

	std::cout << "Socket established\n";

	int sock = accept(sockfd, (struct sockaddr*) &address, &address_len);
	connections.push_back(sock);
	std::cout << "Connected new client\n";

	//Here be dragons

	int valread = 0;
	char recBuffer[1024] = { 0 };
	size_t len_recBuffer = 1024;

	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(20); // (0-100)
	std::vector<uchar> encodeBuffer;
	size_t len_encodeBuffer;

	while (!terminationRequest) {
		valread = read(sock, recBuffer, len_recBuffer);
		if (valread == 0) {
			std::cout << "Client disconnected\n";
			terminationRequest = true;
			break;
		}

//		cv::imencode(".jpg", currentFrame, encodeBuffer, compression_params);
//		cv::Mat m;
//		cv::imdecode(encodeBuffer, 0, &m);
//		cv::imshow("img", m);
//		len_encodeBuffer = encodeBuffer.size();

		currentFrame = currentFrame.reshape(0, 1);
		len_encodeBuffer = currentFrame.total() * currentFrame.elemSize();

		//std::cout << currentFrame.size() << "\n";	//504 x 216

//		send(sock, &len_encodeBuffer, sizeof(len_encodeBuffer), 0);

		uchar* current = currentFrame.data;
		size_t sent = 0;
		size_t toSend = len_encodeBuffer;
		while (toSend > 0) {
			valread = send(sock, current, toSend, 0);
			if (valread < 0) {
				std::cout << "Error while sending frame\n" << errno << "\n";
				terminationRequest = true;
				break;
			}
			toSend -= valread;
			sent += valread;
			current += valread;
			std::cout << "Sent: " << sent << " / " << len_encodeBuffer
					<< " bytes\n";
		}
	}
}

void VideoSocket::consumeFrame(cv::Mat frame) {
	frame.copyTo(currentFrame);
}

