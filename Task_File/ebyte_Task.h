/*
 *ebyte_Task.h
 *
 *  Created on: Feb 10, 2026
 *      Author: kemazhu
 */


#ifndef EBYTE_TASK_H_
#define EBYTE_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "shared_data.h" // g_temperature, etc.

// ID de ruche (défini dans shared_data.h ou ici)
#ifndef HIVE_ID
#define HIVE_ID 1
#endif

void vTaskEBYTE(void *pvParameters);
#endif /* EBYTE_TASK_H_ */
