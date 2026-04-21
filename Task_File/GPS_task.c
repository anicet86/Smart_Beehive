/*
 * GPS_task.c
 *
 *  Created on: Feb 1, 2026
 *      Author: kemazhu
 */


// gps_task.c
#include "shared_data.h"
#include "nmea.h"
#include "gps.h"
#include "task.h"



// Callback GPS : alimente le parser NMEA
static void gps_callback(uint8_t c)
{
    nmea_parse(c); // Remplit nmea_buffer et décode vers gga_data
}

void vTaskGPS(void *pvParameters)
{
    // Initialisation
    nmea_init();           // Réinitialise le parser
    gps_SetCallback(gps_callback); // Enregistre le callback
    Gps_Init();            // Configure USART3 + DMA/ISR

    while(1)
    {
        // Lecture sécurisée du flag
        taskENTER_CRITICAL();
        uint8_t data_ready = gpgga_ready;
        if (data_ready) {
            gpgga_ready = 0; // Acquitter le flag
        }
        taskEXIT_CRITICAL();

        if (data_ready && gga_data.isfixValid) {

            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_latitude = gga_data.lcation.latitude;
                g_longitude = gga_data.lcation.longitude;
                g_lat_hemisphere = gga_data.lcation.NS;
                g_lon_hemisphere = gga_data.lcation.EW;
                g_gps_valid = gga_data.isfixValid;
                xSemaphoreGive(xSensorMutex);
            }
        }

        // Contrôle de fréquence
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
