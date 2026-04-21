/*
 * GPS_task.h
 *
 *  Created on: Feb 1, 2026
 *      Author: kemazhu
 */

#ifndef GPS_TASK_H_
#define GPS_TASK_H_
#include "FreeRTOS.h"
#include "task.h"

// declaration of the function read dht22 sensor
void vTaskGPS(void *pvParameters);



#endif /* GPS_TASK_H_ */
