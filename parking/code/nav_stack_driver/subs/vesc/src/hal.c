#include "hal.h"
#include "datagrams.h" 

#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>


int sendDataUSB(uint8_t* payload, uint16_t length);
int readDataUSB(uint8_t* payload, uint16_t length);

int fd = -1;

int sendData(uint8_t* payload, uint16_t length) 
{
	if (fd == -1)
	{	
		perror("sendData: fd not available\n");
		return -1;
	}

	sendDataUSB(payload, length); 
	length = readDataUSB(payload, length);

	return unpack(payload,length);
}

int sendDataNoReply(uint8_t* payload, uint16_t length) 
{
	if (fd == -1)
	{	
		perror("sendDataNoReply: Connect the battery!!!\n");
		return -1;
	}

	return sendDataUSB(payload, length);
}


int initUSB(char * device, speed_t baudrate)
{

	fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if(fd < 0) {
                printf("Error opening %s: %s\n", device, strerror(errno));
                return -1;
        }

	struct termios options;

	if (tcgetattr(fd, &options) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}
	
#ifdef DEBUG
	printf("default options:\n");
	printf(".c_cflags: %i\n", options.c_cflag);
	printf(".c_iflags: %i\n", options.c_iflag);
	printf(".c_oflags: %i\n", options.c_oflag);
	printf(".c_lflags: %i\n", options.c_lflag);
	printf(".c_cc: %i\n", options.c_cc);
#endif

	// Baudrate
	cfsetospeed(&options, baudrate);
	cfsetispeed(&options, baudrate);

	// 8 Bit Characters
	options.c_cflag |= CS8;

	// No Partity Bits
	options.c_cflag &= ~PARENB;

	// Only one Stop Bit (not two)
	options.c_cflag &= ~CSTOPB;

	// No Flow-Control
	options.c_cflag &= ~CRTSCTS;

	/* setup for non-canonical mode */
	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 1;

#ifdef DEBUG
	printf("\n");
	printf("new options:\n");
	printf(".c_cflags: %i\n", options.c_cflag);
	printf(".c_iflags: %i\n", options.c_iflag);
	printf(".c_oflags: %i\n", options.c_oflag);
	printf(".c_lflags: %i\n", options.c_lflag);
	printf(".c_cc: %i\n", options.c_cc);
#endif

	if (tcsetattr(fd, TCSANOW, &options) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
	return -1;
	}

	return 0;
}


int sendDataUSB(uint8_t* payload, uint16_t length) 
{

	if (fd == -1)
	{
		
		perror("sendDataUSB: fd not available\n");
		return -1;
	}


	if(write(fd, payload, length) == -1)
	{
		return -1;
	}
	return 0;
}

int readDataUSB(uint8_t* payload, uint16_t length) 
{
	do{
		if(read(fd, payload, 1) != 1)
		{
			perror("readDataUSB: read error\n");
			return -2;
		}
	}while(payload[0] != FRAME_START_SHORT && payload[0] != FRAME_START_LONG);


	uint16_t in_length = 0;
	if(payload[0] == FRAME_START_SHORT)
	{
		if(read(fd, payload, 1) != 1)
		{
			perror("readDataUSB: read error\n");
			return -1;
		}
		in_length = payload[0];
	}
	else
	if(payload[0] == FRAME_START_LONG)
	{
		if(read(fd, payload, 2) != 2)
		{
			perror("readDataUSB: read error\n");
			return -1;
		}
		in_length = payload[0] <<8 | payload[1];
	}

	in_length +=3 ;
	uint16_t length_tmp = in_length;
	uint16_t bytes_read = 0;

	do
	{
		bytes_read = read(fd, payload + bytes_read, in_length);
		if(bytes_read <= in_length)
			in_length -= bytes_read;
	} while(in_length);
	return length_tmp;
}
