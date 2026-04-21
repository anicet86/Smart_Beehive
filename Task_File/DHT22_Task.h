/*
 * DHT22_Task.h
 *
 *  Created on: Nov 29, 2025
 *      Author: kemazhu
 */

#ifndef DHT22_TASK_H_
#define DHT22_TASK_H_

#include "FreeRTOS.h"
#include "task.h"

// declaration of the function read dht22 sensor
void vTaskDHT22(void *pvParameters);

#endif /* DHT22_TASK_H_ */
