/*******************************************************************************
 * Copyright (c) 2018 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 * Contributors:
 *    Constantin Dresel - initial API and implementation
 *******************************************************************************/
#include "control_center_downstream.h"
#include "debugprint.h"
#include "listutil.h"
#include <stdio.h>
#include "af3.h"
#include <pthread.h>
#include <signal.h> //for sig_atomic_t
#include "protocol_control_center.h"
#include "debugprint.h"

struct {
	list_t* p_downstream_list;
	list_iterator_t* p_downstream_list_iterator;
	pthread_mutex_t input_mutex;
	pthread_t listener_thread_id;
	volatile sig_atomic_t listener_thread_run;
}control_center_downstream;

static const char noval_string[] = "NoVal";  /*!<  String that represents NoVal.
												Case does not matter here. */
static const char true_string[] = "true";  /*!<  String that represents true.
												Case does not matter here. */
static const char false_string[] = "false";  /*!<  String that represents false.
												Case does not matter here. */

/**
 * \brief Receive messages from control center and pass them to a handler.
 * This function is intended to be executed in a separate pthread thread. It
 * waits for the reception of messages sent from the control center. At
 * reception of a message the given handler function is executed with the
 * received message as parameter value.
 * @param listener_handler A pointer to a handler function to be executed at the
 * reception of a message. The message is passed to this function as a
 * receive_message_t struct. The handler function pointed to should only take
 * this one parameter and should be of return type void.
 */
void* cc_receive_thread_func( void *listener_handler ) {
	void (*message_handler) (receive_message_t) =
			(void (*)(receive_message_t)) listener_handler;
	 ssize_t res = 0;
	char input_buffer[CC_DOWNSTREAM_MAX_MSG_LEN];
	receive_message_t receive_message;

	while(control_center_downstream.listener_thread_run != 0){
		res = protocol_control_center_receive(input_buffer,
				sizeof(input_buffer));
		if(res > 0){
			//everything worked fine - store and handle the received msg now
			sprintf(receive_message.modulename, strtok(input_buffer,"#"));
			sprintf(receive_message.variablename, strtok(NULL,"="));
			sprintf(receive_message.value, strtok(NULL,"@"));
			sprintf(receive_message.time, strtok(NULL,"@"));
			message_handler(receive_message);
		}
		else if(res < 0) {
			//error occurred
			debug_print(DEBUG_PRINT_LEVEL_FEW,"CC DOWNSTREAM: Receiving failed."
					" Next try in 1 Second.\n");
			sleep(1);
		}
		else {
			//no error but received msg with len 0/EOF respectively
			debug_print(DEBUG_PRINT_LEVEL_FEW, "CC DOWNSTREAM: Received cc "
					"downstream message with len 0/EOF.\n Next try in 1 Second."
					"\nModify cc_receive_thread_func() if you did send zero "
					"length message(s) on purpose.\n");
			sleep(1);
		}
	}
	return NULL;
}


/**
 * \brief Create a listener thread for received messages.
 * Creates a listener thread for received messages that listens for downstream
 * messages and executes the given handler function with the received message
 * as parameter value upon message reception.
 * @param listener_handler A pointer to a handler function to be executed at the
 * reception of a message. The message is passed to this function as a
 * receive_message_t struct. The handler function pointed to should only take
 * this one parameter and should be of return type void.
 */
void cc_listen(void (*listener_handler) (receive_message_t)){
	pthread_create(&(control_center_downstream.listener_thread_id),
			NULL, cc_receive_thread_func, listener_handler);
}

/**
 * \brief Get the element with the given name form all received messages.
 * If an Element with the given name is present in the list of all received
 * messages it is popped from the list and stores to the given destination
 * variable. If no element with the given name can be found in the list the
 * value of the destination variable will remain unchanged.
 * @param name The name of the variable to find in and pop from the list.
 * @param p_target_variable A pointer of the type receive_message_t pointing to
 * the destination where a found list element will be written to if names
 * matched.
 * @return Positive if an element with the requested name was found, negative
 * otherwise.
 */
int get_from_rx_list(char* name, receive_message_t* p_target_variable) {
	int res = -1;
	rx_msg_container_t* p_current_element = NULL;

	if(list_util_is_empty(control_center_downstream.p_downstream_list) != false)
	{
		return res;
	}
	list_util_iterator_reset(
			control_center_downstream.p_downstream_list_iterator, true);
	pthread_mutex_lock(&(control_center_downstream.input_mutex));
	//crawl list of rx'ed messages
	while(list_util_iterator_has_next(
			control_center_downstream.p_downstream_list_iterator)) {
		p_current_element = list_util_iterator_current(
				control_center_downstream.p_downstream_list_iterator);
		if(strcmp(name,	(const char*) p_current_element->message.variablename)
				== 0) {
			p_current_element = list_util_remove_element(
					control_center_downstream.p_downstream_list_iterator);
			*p_target_variable = p_current_element->message;
			list_util_iterator_reset(
					control_center_downstream.p_downstream_list_iterator, true);
			free(p_current_element);
			res = 1;
			continue; /* continue is placed here to start crawling the list
			again. This ensures that always the most recently received value
			for a particular variable is extracted from the list of received
			messages. If you want to change that behavior such that list entries
			related to one variable are found and popped from the list one at a
			time in chronological order of their receiving for every call to
			this function just replace continue by break*/
		}
		p_current_element = list_util_iterator_next(
				control_center_downstream.p_downstream_list_iterator);
	}
	pthread_mutex_unlock(&(control_center_downstream.input_mutex));
	return res;
}

/**
 * \brief Add a given message to the list of received messages.
 * Adds a given message to the list of all received and yet unhandled messages.
 * @param p_receive_message_to_put A pointer of the type receive_message_t
 * pointing to a message to newly add to the list of received messages.
 * @return A pointer to the newly added element in the list of received
 * messages. Null if memory allocation failed.
 */
rx_msg_container_t* put_to_rx_list(receive_message_t* p_receive_message_to_put)
{
	rx_msg_container_t* p_new_msg_to_store;
	p_new_msg_to_store = malloc(sizeof(rx_msg_container_t));

	if(p_new_msg_to_store == NULL){
		debug_print(DEBUG_PRINT_LEVEL_FEW, "CC DOWNSTREAM: Memory allocation "
				"failed when receiving downstream message.\nSkipping.\n");
	}
	else {
		p_new_msg_to_store->message = *p_receive_message_to_put;
		p_new_msg_to_store->age_in_cycles = 0;
		pthread_mutex_lock(&(control_center_downstream.input_mutex));
		list_util_append(control_center_downstream.p_downstream_list,
				p_new_msg_to_store);
		pthread_mutex_unlock(&(control_center_downstream.input_mutex));
	}
	return p_new_msg_to_store;
}

/**
 * \brief Print the content of a receive_message_t structure.
 * Prints the content of a given receive_message_t structure as readable string
 * representation. CAUTION: global_debug_print_level has to be set to
 * DEBUG_PRINT_LEVEL_FEW or something more verbose for this function to produce
 * any output.
 * @param receive_message The message to print the content from.
 */
void debug_print_receive_message( receive_message_t receive_message){
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Received Downstream message with"
			"content:\n");
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Modulname: %s\n",
			receive_message.modulename);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Variablename: %s\n",
			receive_message.variablename);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Value: %s\n", receive_message.value);
	debug_print(DEBUG_PRINT_LEVEL_FEW, "Time: %s\n", receive_message.time);
}

/**
 * \brief A function to handle received downstream messages one at a time.
 * This function takes care of storing the message handed to it in the list
 * buffer for received messages and optionally causes the message content to
 * be printed in a readable format.
 * @param received_message The message of type receive_message_t to be handled.
 */
void cc_downstream_message_handler(receive_message_t received_message) {
	put_to_rx_list(&received_message);
	debug_print_receive_message(received_message); //DEBUG
}

/**
 * \brief Checks whether a string resembles NoVal.
 * This function checks whether a specified string is equal to NoVal. Case of
 * the particular letters within the string does not matter.
 * @param string_to_check A pointer to the first char of the string to analyze.
 * @return 1 if the string represents NoVal 0 otherwise.
 */
static inline uint8_t is_noval(char* string_to_check) {
	//we want to return true for noval found false otherwise
	if(strcasecmp(string_to_check, noval_string) == 0) { //noval found
		return 1; //true
	}
	return 0; //false
}

/**
 * \brief Checks whether a string resembles true.
 * This function checks whether a specified string is equal to true. Case of
 * the particular letters within the string does not matter.
 * @param string_to_check A pointer to the first char of the string to analyze.
 * @return 1 if the string represents true 0 otherwise.
 */
static inline uint8_t is_true(char* string_to_check) {
	//we want to return true for true found false otherwise
	if(strcasecmp(string_to_check, true_string) == 0) { //true found
		return 1; //true
	}
	return 0; //false
}

/**
 * \brief Checks whether a string resembles false.
 * This function checks whether a specified string is equal to false. Case of
 * the particular letters within the string does not matter.
 * @param string_to_check A pointer to the first char of the string to analyze.
 * @return 1 if the string represents false 0 otherwise.
 */
static inline uint8_t is_false(char* string_to_check) {
	//we want to return true for false found false otherwise
	if(strcasecmp(string_to_check, false_string) == 0) { //false found
		return 1; //true
	}
	return 0; //false
}

int af3_cc_get_int(char* userfriendly_name, GEN_TYPE_int* p_variable_to_write,
		GEN_TYPE_boolean* p_variable_to_write_noval) {
	int res = -1;
	receive_message_t found_message;

	res = get_from_rx_list(userfriendly_name, &found_message);

	if(res > 0) { //variable successfully found
		if(is_noval(found_message.value) > 0){
			*p_variable_to_write_noval = true;
		}
		else {
			//alter value of variable to write
			*p_variable_to_write = (GEN_TYPE_int) strtol(found_message.value,
					NULL, CC_DOWNSTREAM_INTBASE);
			*p_variable_to_write_noval = false; //var now contains valid value
		}
	}
	return res;
}

int af3_cc_get_bool(char* userfriendly_name,
		GEN_TYPE_boolean* p_variable_to_write,
		GEN_TYPE_boolean* p_variable_to_write_noval) {
	int res = -1;
	receive_message_t found_message;

	res = get_from_rx_list(userfriendly_name, &found_message);

	if(res > 0) { //variable successfully found
		if(is_noval(found_message.value) > 0){
			*p_variable_to_write_noval = true;
		}
		else if(is_true(found_message.value) > 0) {
			*p_variable_to_write = true;
			*p_variable_to_write_noval = false; //var now contains valid value
		}
		else if(is_false(found_message.value) > 0) {
			*p_variable_to_write = false;
			*p_variable_to_write_noval = false; //var now contains valid value
		}
		else {
			//alter value of variable to write
			*p_variable_to_write =
					(GEN_TYPE_boolean) strtol(found_message.value, NULL,
							CC_DOWNSTREAM_INTBASE);
			*p_variable_to_write_noval = false; //var now contains valid value
		}
	}
	return res;
}

int af3_cc_get_double(char* userfriendly_name,
		GEN_TYPE_double* p_variable_to_write,
		GEN_TYPE_boolean* p_variable_to_write_noval) {
	int res = -1;
	receive_message_t found_message;

	res = get_from_rx_list(userfriendly_name, &found_message);

	if(res > 0) { //variable successfully found
		if(is_noval(found_message.value) > 0){
			*p_variable_to_write_noval = true;
		}
		else {
			//alter value of variable to write
			*p_variable_to_write =
					(GEN_TYPE_double) strtold(found_message.value, NULL);
			*p_variable_to_write_noval = false; //var now contains valid value
		}
	}
	return res;
}

void af3_cc_donwstream_init(void){
	//allow later created thread to run
	control_center_downstream.listener_thread_run = 0x01;
	//initialize mutex used for transmitter callbacks
	pthread_mutex_init(&(control_center_downstream.input_mutex), NULL);
	//create list and iterator for list
	control_center_downstream.p_downstream_list = list_util_create_list();
	control_center_downstream.p_downstream_list_iterator = list_util_iterator(
					control_center_downstream.p_downstream_list, true);
	//start listening for messages
	cc_listen(cc_downstream_message_handler);
}

void af3_cc_downstream_exit(void) {
	//stop listener thread
	control_center_downstream.listener_thread_run = 0x00;
	//Listener thread may be in blocking recv() so just joining might be no use.
	pthread_cancel(control_center_downstream.listener_thread_id);
	pthread_join(control_center_downstream.listener_thread_id, NULL);
	list_util_iterator_destroy(
			control_center_downstream.p_downstream_list_iterator);
	list_util_destroy_list(control_center_downstream.p_downstream_list);
	pthread_mutex_destroy(&(control_center_downstream.input_mutex));
}

void af3_cc_downstream_clean_up_rx_list(uint8_t age_to_keep) {
	rx_msg_container_t* p_current_element = NULL;

	if(list_util_is_empty(control_center_downstream.p_downstream_list) != false)
	{
		return; //noting to do here
	}
	list_util_iterator_reset(
			control_center_downstream.p_downstream_list_iterator, true);
	pthread_mutex_lock(&(control_center_downstream.input_mutex));
	//crawl list of rx'ed messages
	while(list_util_iterator_has_next(
			control_center_downstream.p_downstream_list_iterator)) {
		p_current_element = list_util_iterator_current(
				control_center_downstream.p_downstream_list_iterator);
		if(p_current_element->age_in_cycles > age_to_keep) {
			p_current_element = list_util_remove_element(
					control_center_downstream.p_downstream_list_iterator);
			list_util_iterator_reset(
					control_center_downstream.p_downstream_list_iterator,true);
			free(p_current_element);
			continue;
		}
		(p_current_element->age_in_cycles)++;
		p_current_element = list_util_iterator_next(
				control_center_downstream.p_downstream_list_iterator);
	}
	pthread_mutex_unlock(&(control_center_downstream.input_mutex));
	return;
}
