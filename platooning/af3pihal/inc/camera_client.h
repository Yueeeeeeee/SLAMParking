/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Sebastian Neubauer, Florian Hoelzl - initial API and implementation
 *******************************************************************************/
 
#ifndef INC_CAMERA_CLIENT_H_
#define INC_CAMERA_CLIENT_H_

#include <unistd.h>

#include "debugprint.h"

#include <stdbool.h>
#include <stdint.h>

/** Initialize the connection to the camera server. */
void camera_client_initialize(const char *hostname, const char *port);

/** Returns whether the client is connected to the camera. */
bool camera_client_is_connected();
/** Returns the left distance. */
float camera_client_get_distance_left();
/** Returns the right distance. */
float camera_client_get_distance_right();
/** Returns the yaw angle. */
float camera_client_get_yaw_angle();
/** Returns the detection state left. */
bool camera_client_get_detection_state_left();
/** Returns the detection state left. */
bool camera_client_get_detection_state_right();
/** Returns the number of milliseconds since the last camera data was received. */
uint64_t camera_client_get_ms_since_last_update();

/** Returns whether a preceding rover has been recognized. */
bool camera_client_is_preceding_rover_recognized();
/** Returns the distance to the preceding rover in mm. */
int camera_client_get_preceding_rover_distance();
/** Returns the QR code id of the preceding rover. */
int camera_client_get_preceding_rover_id();
/** Returns whether a curve was detected. */
bool camera_client_is_curve_detected();
/** Returns the radius of the detected curve. */
float camera_client_get_curve_radius();

#endif /* INC_CAMERA_CLIENT_H_ */
