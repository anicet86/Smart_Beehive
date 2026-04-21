/*
 * DHT22_Task.c
 *
 *  Created on: Nov 29, 2025
 *      Author: kemazhu
 */

#include "shared_data.h"
#include "DHT22_Task.h"
#include "dht22.h"
#include "task.h"
#include "stm32f407xx.h"
#include "semphr.h"
#include <stdio.h>



void vTaskDHT22(void *pvParameters)
{
DHT22_Data dht22;

     DHT22_Init();   // Initialisation du dht22 sensor



	while(1)
	{
		if(DHT22_Read(&dht22) == 0)
		{
			if(xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE)//(xSemaphoreTake(xSensorMutex, 0) == pdTRUE)//(xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
			{
                g_temperature = dht22.dht_temperature;
                g_humidity = dht22.dht_humidity;

                xSemaphoreGive(xSensorMutex);
			}
		//	printf("DHT22: T=%.2f°C, H=%.1f%%\n", dht22.dht_temperature, dht22.dht_humidity);
		}
		else
		{
		//	printf("DHT22: Échec de lecture\n");
		}

		//vTaskDelay(pdMS_TO_TICKS(2000));
	  // Attendre 2 secondes (DHT22 max ~0.5 Hz)
	 // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		vTaskDelay(pdMS_TO_TICKS(3000));
  }

}

