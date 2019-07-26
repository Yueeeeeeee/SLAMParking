#ifndef HAL_H_
#define HAL_H_

#include <stdint.h>
#include <termios.h>

enum {
    USB = 0,
    CAN,
    UART,
    PPM
};

int initUSB(char* device, speed_t baudrate);

int sendData(uint8_t* payload, uint16_t length);
int sendDataNoReply(uint8_t* payload, uint16_t length);


#endif /* HAL_H_ */
