/*
 * debug_task.h
 *
 *  Created on: Jan 6, 2026
 *      Author: kemazhu
 */

#ifndef DEBUG_TASK_H_
#define DEBUG_TASK_H_

#include "FreeRTOS.h"
#include "task.h"

void vTaskDebugSerial(void *pvParameters);


#endif /* DEBUG_TASK_H_ */
