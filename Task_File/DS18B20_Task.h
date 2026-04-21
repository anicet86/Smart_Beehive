/*
 * DS18B20_Tasck.h
 *
 *  Created on: Dec 1, 2025
 *      Author: kemazhu
 */

#ifndef DS18B20_TASK_H_
#define DS18B20_TASK_H_

#include "FreeRTOS.h"
#include "task.h"


void vTaskDS18B20(void * pvParameters);


#endif /* DS18B20_TASK_H_ */
