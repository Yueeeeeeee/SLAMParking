/*******************************************************************************
 * Copyright (c) 2017 fortiss GmbH.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v20.html
 *
 * Contributors:
 *    Florian Hoelzl - initial API and implementation
 *    Constantin Dresel - added destroy_list function
 *******************************************************************************/
#include "listutil.h"
#include <stdlib.h>
#include <stdio.h>

typedef struct list_entry {
	void* element;
	void* next;
	void* previous;
} list_entry_t;

struct list {
	list_entry_t* head;
	list_entry_t* tail;
};

static inline void out_of_memory() {
	perror("Out of memory during malloc() in af3_component.c.");
}

/** Creates a new list with the given element size. */
list_t* list_util_create_list() {
	list_t* l = malloc(sizeof(list_t));
	if(l == NULL) {
		out_of_memory();
		return NULL;
	}
	l->tail = NULL;
	l->head = NULL;
	return l;
}

void list_util_destroy_list(list_t* list) {
	//remove all list entries and free memory space
	/*This will NOT affect the memory chunks occupied by the actual elements
	 * that the list entries pointed to. */
	while(list_util_is_empty(list)!=true){
		list_util_remove_first(list);
	}
	free(list);
}

static list_entry_t* alloc_list_entry(void* element) {
	list_entry_t* entry = malloc(sizeof(list_entry_t));
	if(entry == NULL) {
		out_of_memory();
		return NULL;
	}
	entry->element = element;
	entry->previous = NULL;
	entry->next = NULL;
	return entry;
}

void list_util_append(list_t* list, void* element) {
	list_entry_t* entry = alloc_list_entry(element);
	if(entry != NULL) {
		if(list->tail != NULL) {
			list->tail->next = entry;
			entry->previous = list->tail;
			list->tail = entry;
		} else {
			list->head = entry;
			list->tail = entry;
		}
	}
}

void* list_util_remove_last(list_t* list) {
	if(list->tail == NULL) {
		return NULL;
	}
	if(list->head == list->tail) {
		void* result = list->head->element;
		free(list->head);
		list->head = NULL;
		list->tail = NULL;
		return result;
	}
	list_entry_t* last = list->tail;
	void* result = last->element;
	list->tail = last->previous;
	free(last);
	return result;
}

void list_util_prepend(list_t* list, void* element) {
	list_entry_t* entry = alloc_list_entry(element);
	if(entry != NULL) {
		if(list->head != NULL) {
			list->head->previous = entry;
			entry->next = list->head;
			list->head = entry;
		} else {
			list->head = entry;
			list->tail = entry;
		}
	}
}

void* list_util_remove_first(list_t* list) {
	if(list->head == NULL) {
		return NULL;
	}
	if(list->head == list->tail) {
		void* result = list->head->element;
		free(list->head);
		list->head = NULL;
		list->tail = NULL;
		return result;
	}
	list_entry_t* first = list->head;
	void* result = first->element;
	list->head = first->next;
	free(first);
	return result;
}

bool list_util_is_empty(list_t* list) {
	return list->head == NULL  && list->tail == NULL;
}


struct list_iterator {
	list_t* list;
	list_entry_t* current;
};

list_iterator_t* list_util_iterator(list_t* list, bool forward) {
	list_iterator_t* iter = malloc(sizeof(list_iterator_t));
	if(iter == NULL) {
		out_of_memory();
		return NULL;
	}
	iter->list = list;
	if(forward) {
		iter->current = list->head;
	} else {
		iter->current = list->tail;
	}
	return iter;
}

void list_util_iterator_reset(list_iterator_t* iter, bool start) {
	if(start) {
		iter->current = iter->list->head;
	} else {
		iter->current = iter->list->tail;
	}
}

bool list_util_iterator_has_next(list_iterator_t* iter) {
	return iter->current != NULL;
}

void* list_util_iterator_next(list_iterator_t* iter) {
	if(iter->current == NULL) {
		return NULL;
	}
	void* result = iter->current->element;
	iter->current = iter->current->next;
	return result;
}

void* list_util_iterator_current(list_iterator_t* iter) {
	if(iter->current == NULL) {
		return NULL;
	}
	void* result = iter->current->element;
	return result;
}

bool list_util_iterator_has_previous(list_iterator_t* iter) {
	return iter->current != NULL;
}

void* list_util_iterator_previous(list_iterator_t* iter) {
	if(iter->current == NULL) {
		return NULL;
	}
	void* result = iter->current->element;
	iter->current = iter->current->previous;
	return result;
}

void* list_util_remove_element(list_iterator_t* list_iterator) {
	void* result = NULL;
	if(list_iterator->current == list_iterator->list->head) {
		//first list element
		result = list_util_remove_first(list_iterator->list);
	}
	else if(list_iterator->current == list_iterator->list->tail) {
		//last list element
		result = list_util_remove_last(list_iterator->list);
	}
	else {
		//not first and not last list entry
		if(list_iterator->list->head == NULL) {
			result = NULL;
		}
		result = list_iterator->current->element;
		list_entry_t* p_prev_entry;
		list_entry_t* p_next_entry;
		p_next_entry = (list_entry_t*) list_iterator->current->next;
		p_prev_entry = (list_entry_t*) list_iterator->current->previous;
		p_next_entry->previous = list_iterator->current->previous;
		p_prev_entry->next = list_iterator->current->next;
		free(list_iterator->current);
	}
	return result;
}

void list_util_iterator_destroy(list_iterator_t* iter) {
	free(iter);
}
