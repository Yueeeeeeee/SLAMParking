/*
 * VideoDisplay.cpp
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#include "ResultServer.h"

ResultServer::ResultServer() {
init_server();
}

ResultServer::~ResultServer() {

}

void ResultServer::consumeResultData(LDValues currentValues) {
send_values(&currentValues);
}
