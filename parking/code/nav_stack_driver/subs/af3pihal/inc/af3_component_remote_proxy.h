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

#ifndef INC_AF3_COMPONENT_REMOTE_PROXY_H_
#define INC_AF3_COMPONENT_REMOTE_PROXY_H_

#include "af3_component.h"
/** Initializes the AF3 remote component proxy module and returns true if required data structures could be allocated. */
bool af3_component_remote_proxy_module_initialize();

/**
 * Returns a port that can be linked to AF3 component input ports, which are then
 * automatically filled when the remote input is filled.
 */
af3_port_t* af3_component_remote_proxy_input_create(uint8_t remote_unit_id, uint16_t signal_id, char* name, uint16_t type);

/**
 * Consumes the given CAN message and returns true if it was successfully handled.
 * The message will not be freed.
 */
bool af3_component_remote_proxy_handle_input(uint8_t remote_unit_id, uint16_t signal_id, void* value);

/** Clears the input buffers of the remote proxy components. */
void af3_component_remote_proxy_clear_inputs();

#endif /* INC_AF3_COMPONENT_REMOTE_PROXY_H_ */
