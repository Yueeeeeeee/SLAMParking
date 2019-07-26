/*******************************************************************************
 * Copyright (c) 2018 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Constantin Dresel - initial API and implementation
 *******************************************************************************/
#ifndef INC_CONTROL_CENTER_DOWNSTREAM_H_
#define INC_CONTROL_CENTER_DOWNSTREAM_H_

#include "af3.h"
#include <stdint.h>
#define CC_DOWNSTREAM_INTBASE 10  /*!< Base for conversion from string to int
									(10 for dec) */
#define CC_DOWNSTREAM_MAX_MSG_LEN 500  /*!< The maximum length of downstream
										messages that can be received given in
										byte */

typedef struct rx_msg_container_t {
	receive_message_t message;
	uint8_t age_in_cycles;
}rx_msg_container_t;  /*!< a container for a received message and the message
						age given in cycles of the main loop since reception */

/**
 * \brief Initialize the downstream communication for control center.
 * Currently just initializes a mutex and starts a thread listening for incoming
 * messages.
 */
void af3_cc_donwstream_init(void);

/**
 * \brief Exit downstream communication for control center.
 * Stops the thread used for receiving incoming downstream messages and frees
 * used memory.
 */
void af3_cc_downstream_exit(void);

/**
 * \brief Handler function for received control center downstream messages.
 * Stores a given received message in a list of received messages for further
 * processing.
 * @param received_message The received message to be handled.
 */
void cc_downstream_message_handler(receive_message_t received_message);

/**
 * \brief Get a received integer variable with the given name.
 * Checks whether an integer variable with the given name was received. If so
 * the value of the received variable is converted to integer an stored at the
 * given destination. If no appropriate message was received the destination
 * variable won't get modified.
 * @param userfriendly_name The userfriendly name of the variable to search for.
 * @param p_variable_to_write A pointer to the destination where the integer
 * value will be stored if a message was received containing the desired
 * variable.
 * @param p_variable_to_write A pointer to the destination where to record
 * whether the variable is valid or not.
 * @return Positive if the destination variable was updated, negative otherwise.
 */
int af3_cc_get_int(char* userfriendly_name, GEN_TYPE_int* p_variable_to_write,
		GEN_TYPE_boolean* p_variable_to_write_noval);

/**
 * \brief Get a received boolean variable with the given name.
 * Checks whether an boolean variable with the given name was received. If so
 * the value of the received variable is converted to boolean an stored at the
 * given destination. If no appropriate message was received the destination
 * variable won't get modified.
 * @param userfriendly_name The userfriendly name of the variable to search for.
 * @param p_variable_to_write A pointer to the destination where the boolean
 * value will be stored if a message was received containing the desired
 * variable.
 * @param p_variable_to_write A pointer to the destination where to record
 * whether the variable is valid or not.
 * @return Positive if the destination variable was updated, negative otherwise.
 */
int af3_cc_get_bool(char* userfriendly_name,
		GEN_TYPE_boolean* p_variable_to_write,
		GEN_TYPE_boolean* p_variable_to_write_noval);

/**
 * \brief Get a received double variable with the given name.
 * Checks whether an double variable with the given name was received. If so
 * the value of the received variable is converted to double an stored at the
 * given destination. If no appropriate message was received the destination
 * variable won't get modified.
 * @param userfriendly_name The userfriendly name of the variable to search for.
 * @param p_variable_to_write A pointer to the destination where the double
 * value will be stored if a message was received containing the desired
 * variable.
 * @param p_variable_to_write A pointer to the destination where to record
 * whether the variable is valid or not.
 * @return Positive if the destination variable was updated, negative otherwise.
 */
int af3_cc_get_double(char* userfriendly_name,
		GEN_TYPE_double* p_variable_to_write,
		GEN_TYPE_boolean* p_variable_to_write_noval);
/**
 * \brief Clear received messages that are older than a given number of cycles.
 * Clears received messages stored in the list buffer that were not popped from
 * the list within the given interval of cycles. Use this to clear received
 * messages from the list that are not related to an existing variable and
 * therefore would pile up in the list and never be popped otherwise.
 * One call of this function represents one cycle.
 * @param age_to_keep The maximum age of messages to keep given in cycles.
 */
void af3_cc_downstream_clean_up_rx_list(uint8_t age_to_keep);


#endif /* INC_CONTROL_CENTER_DOWNSTREAM_H_ */
