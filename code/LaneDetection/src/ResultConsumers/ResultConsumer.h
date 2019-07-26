/*
 * ResultConsumer.h
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#ifndef SRC_RESULTCONSUMERS_RESULTCONSUMER_H_
#define SRC_RESULTCONSUMERS_RESULTCONSUMER_H_

#include "../ClientServer/Data.h"

class ResultConsumer {
public:
	ResultConsumer();
	virtual ~ResultConsumer();
	virtual void consumeResultData(LDValues currentValues)=0;
};

#endif /* SRC_RESULTCONSUMERS_RESULTCONSUMER_H_ */
