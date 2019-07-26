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
#ifndef INC_DEBUGPRINT_H_
#define INC_DEBUGPRINT_H_

#define DEBUG_PRINT_LEVEL_NONE	0
#define DEBUG_PRINT_LEVEL_FEW	1
#define DEBUG_PRINT_LEVEL_MANY	2
#define DEBUG_PRINT_LEVEL_ALL	3

extern int global_debug_print_level;

#ifdef DEBUG
	#define USE_DEBUG_PRINTING 1
#else
	#define USE_DEBUG_PRINTING 0
#endif

#define debug_print(lvl, fmt, ...) \
	if(USE_DEBUG_PRINTING) if(lvl <= global_debug_print_level) fprintf(stderr, fmt, ##__VA_ARGS__)

#endif /* INC_DEBUGPRINT_H_ */
