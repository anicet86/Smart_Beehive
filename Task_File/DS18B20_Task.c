/*
 * DS18B20_Task.c
 *
 *  Created on: Dec 1, 2025
 *      Author: kemazhu
 */

#include <shared_data.h>
#include "DS18B20_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "DS18B20.h"
#include <stdio.h>

void vTaskDS18B20(void *pvParameters)
{
    DS18B20_Data ds18b20;
  DS18B20_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Désactive les interruptions
        taskENTER_CRITICAL();
        int result = DS18B20_ReadTemperature(&ds18b20);
        taskEXIT_CRITICAL();

        if (result == DS18B20_OK)
        {
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                g_temperature2 = ds18b20.temperature;
                xSemaphoreGive(xSensorMutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}
