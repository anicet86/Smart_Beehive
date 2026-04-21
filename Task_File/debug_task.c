/*
 * debug_task.c
 *
 *  Created on: Jan 6, 2026
 *      Author: kemazhu
 */

#include "debug_task.h"
#include "shared_data.h"
#include <stdio.h>
#include "nmea.h"
void vTaskDebugSerial(void *pvParameters)
{
	while(1)
	{
		float DHT22_Temperature , DHT22_Humidity,DS18B20_Temperature ;
		if (xSemaphoreTake(xSensorMutex , pdMS_TO_TICKS(50)) == pdTRUE)
		{

			DHT22_Temperature = g_temperature;
			DHT22_Humidity = g_humidity;
			DS18B20_Temperature = g_temperature2;
			xSemaphoreGive(xSensorMutex );
		}

		//affichage dans le terminal serie

		printf("****** DATA SMART BEEHIVE ******\n");
		printf("Temperature intern: DHT22_Temperatur= %.2f\n",DHT22_Temperature);
		printf("Humidity intern: DHT22_Humidity=%.2f\n",DHT22_Humidity);
		printf("Temperature extern: DS18B20_Temperature=%.2f\n",DS18B20_Temperature);
		printf("GPGGA Raw: %s\n", nmea_buffer);
		printf("Lat: %.6f %c, Lon: %.6f %c, Valid: %d\n",
		       g_latitude, g_lat_hemisphere,
		       g_longitude, g_lon_hemisphere,
		       g_gps_valid);
		printf("son = %.2f\n",g_bee_tone_energy);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}
