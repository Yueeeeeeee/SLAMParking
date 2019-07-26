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
#ifndef INC_TIMEUTIL_H_
#define INC_TIMEUTIL_H_
#include <stdint.h>

#define MICROS_IN_NANOS		(uint64_t)1000
#define MILLIS_IN_MICROS	(uint64_t)1000
#define SECONDS_IN_MILLIS	(uint64_t)1000

#define MILLIS_IN_NANOS MILLIS_IN_MICROS * MICROS_IN_NANOS
#define SECONDS_IN_MICROS SECONDS_IN_MILLIS * MILLIS_IN_MICROS
#define SECONDS_IN_NANOS SECONDS_IN_MILLIS * MILLIS_IN_NANOS

#define YEARS_IN_SECONDS (uint64_t)365 * (uint64_t)24 * (uint64_t)3600;

/** Returns the current time in microseconds. */
uint64_t time_util_get_current_micros();

/** Returns the elapsed microseconds since the start time. */
uint64_t time_util_get_elapsed_micros_since(uint64_t start_time);

/** Sleeps for the amount of seconds. */
void time_util_sleep_seconds(uint64_t seconds);

/** Sleeps for the amount of milliseconds. */
void time_util_sleep_millis(uint64_t millis);

/** Sleeps for the amount of microseconds. */
void time_util_sleep_micros(uint64_t micros);

#endif /* INC_TIMEUTIL_H_ */
