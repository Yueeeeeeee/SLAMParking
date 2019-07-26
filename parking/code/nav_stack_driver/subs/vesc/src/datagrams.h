#ifndef DATAGRAMS_H_
#define DATAGRAMS_H_

#include <stdint.h>

#define FRAME_START_SHORT 2
#define FRAME_START_LONG 3
#define FRAME_END 3

int pack(uint8_t* payload, uint16_t length, uint8_t* dataOut );
int unpack(uint8_t* payload, uint16_t length);



#endif /* DATAGRAMS_H_ */
