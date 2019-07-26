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
#ifndef INC_PROTOCOL_COORDINATOR_H_
#define INC_PROTOCOL_COORDINATOR_H_

#include <stdint.h>

#include "protocol_worker.h"
#include "timeutil.h"

/** Main function of coordinator unit. */
int protocol_coordinator_main(
		protocol_can_config_t* can_config,
		protocol_worker_clock_config_t* clock_config,
		protocol_worker_computation_config_t* computation_config,
		list_t* remote_units);

#endif /* INC_PROTOCOL_COORDINATOR_H_ */
