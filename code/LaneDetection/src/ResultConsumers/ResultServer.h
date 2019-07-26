/*
 * VideoDisplay.h
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#ifndef SRC_RESULTCONSUMERS_RESULTSERVER_H_
#define SRC_RESULTCONSUMERS_RESULTSERVER_H_

//this is only a wrapper class...
#include "ResultConsumer.h"

class ResultServer: public ResultConsumer {
public:
	ResultServer();
	virtual ~ResultServer();
	void consumeResultData(LDValues currentValues);
private:

};

#endif /* SRC_RESULTCONSUMERS_RESULTSERVER_H_ */
