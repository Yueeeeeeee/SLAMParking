#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "datagrams.h"
#include "crc.h"


int pack(uint8_t* payload, uint16_t length, uint8_t* dataOut ) 
{
	uint16_t crc = crc16(payload, length);
	uint16_t count = 0;

	if (length < 256)
	{
		dataOut[count++] = FRAME_START_SHORT;
		dataOut[count++] = (uint8_t)length;
	}
	else
	{
		dataOut[count++] = FRAME_START_LONG;
		dataOut[count++] = (uint8_t)(length >> 8);
		dataOut[count++] = (uint8_t)(length & 0xFF);
	}

	memcpy(dataOut+count, payload, length);

	count += length;
	dataOut[count++] = (uint8_t)(crc >> 8);
	dataOut[count++] = (uint8_t)(crc & 0xFF);
	dataOut[count++] = FRAME_END;

	return count;
}


int unpack(uint8_t* payload, uint16_t length) 
{
	uint16_t crc_computed = crc16(payload, length-3);
	uint16_t crc_received = payload[length-3]<<8 | payload[length-2];

	if(crc_computed == crc_received)
		return length-3;
	else
	{
		#warning TODO: request package again if CRC check fails
		return -1;
	}

}


