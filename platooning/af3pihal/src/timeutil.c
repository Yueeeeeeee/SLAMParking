/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *******************************************************************************/
#include "timeutil.h"

#include <time.h>
#include <unistd.h>

uint64_t time_util_get_current_micros() {
	struct timespec now;
	uint64_t result = 0;
	if(clock_gettime(CLOCK_REALTIME, &now) == 0) {
		result = (uint64_t)now.tv_sec * (uint64_t)SECONDS_IN_NANOS;
		result += (uint64_t)now.tv_nsec;
		result /= (uint64_t)MICROS_IN_NANOS;
	}
	return result;
}

uint64_t time_util_get_elapsed_micros_since(uint64_t start_time) {
	return time_util_get_current_micros() - start_time;
}

void time_util_sleep_seconds(uint64_t seconds) {
	usleep(seconds * SECONDS_IN_MICROS);
}

void time_util_sleep_millis(uint64_t millis) {
	usleep(millis * MILLIS_IN_MICROS);
}

void time_util_sleep_micros(uint64_t micros) {
	usleep(micros);
}
