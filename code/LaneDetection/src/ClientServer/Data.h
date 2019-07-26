#ifndef DATA_H
#define DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
	float distanceLeft;
	float distanceRight;
	float yawAngle;
	bool detectedLeft;
	bool detectedRight;
	int precedingRoverDistance; //mm
	int precedingRoverId;
	bool precedingRoverRecognized;
	bool curve;
	float curveRadius;

} LDValues;

typedef struct {
	float distanceLeft;
	float distanceRight;
	float yawAngle;
	bool detectedLeft;
	bool detectedRight;
	bool ldConnected;
	unsigned long msSinceLastUpdate;
} AF3LDValues;

/* Server functions */
void init_server();

// Send values to all connected clients.
void send_values(const LDValues *values);


/* Client functions */
void init_client(const char *hostname, const char *port);

// Get the last cached version of the values.
AF3LDValues get_values();

#ifdef __cplusplus
}
#endif

#endif
