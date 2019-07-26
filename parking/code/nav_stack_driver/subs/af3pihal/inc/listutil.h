/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *    Constantin Dresel - added destroy_list and other functions
 *******************************************************************************/
#ifndef INC_LISTUTIL_H_
#define INC_LISTUTIL_H_

#include "stdbool.h"

typedef struct list list_t;

typedef struct list_iterator list_iterator_t;

/** Creates a new list with the given element size. */
list_t* list_util_create_list();

/** Frees the memory occupied by the given list. */
void list_util_destroy_list(list_t* list);

/** Appends the given element to the end of the list. */
void list_util_append(list_t* list, void* element);

/** Prepends the given element to the begin of the list. */
void list_util_prepend(list_t* list, void* element);

/** Removes and returns the first element in the list. */
void* list_util_remove_first(list_t* list);

/** Removes and returns the last element in the list. */
void* list_util_remove_last(list_t* list);

bool list_util_is_empty(list_t* list);

/** Creates a list iterator for the given list starting at the head or tail. */
list_iterator_t* list_util_iterator(list_t* list, bool forward);

/** Resets the iterator to the beginning or the end of its list. */
void list_util_iterator_reset(list_iterator_t* iter, bool start);

/** Checks whether the given iterator has a next element during forward iteration. */
bool list_util_iterator_has_next(list_iterator_t* iter);

/** Returns the next element during forward iteration and advances the iterator forward. */
void* list_util_iterator_next(list_iterator_t* iter);

/** Returns the current element during iteration without advancing the iterator.
 *  */
void* list_util_iterator_current(list_iterator_t* iter);

/** Checks whether the given iterator has a previous element during backward iteration. */
bool list_util_iterator_has_previous(list_iterator_t* iter);

/** Returns the previous element during backward iteration and advances the iterator backward. */
void* list_util_iterator_previous(list_iterator_t* iter);

/** Destroys the iterator when it is no longer needed. */
void list_util_iterator_destroy(list_iterator_t* iter);

/** Removes and returns the specified element in the list. */
void* list_util_remove_element(list_iterator_t* list_iterator);
#endif /* INC_LISTUTIL_H_ */
