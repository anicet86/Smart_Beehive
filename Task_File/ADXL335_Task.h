/*
 * ADXL335_Tasck.h
 *
 *  Created on: Dec 3, 2025
 *      Author: kemazhu
 */

#ifndef ADXL335_TASK_H_
#define ADXL335_TASK_H_
#include "adxl335_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adxl335.h"

void vTaskADXL335(void *pvParameters);

#endif /* ADXL335_TASK_H_ */
