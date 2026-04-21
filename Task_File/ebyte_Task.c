/*
 * ebyte_Task.c
 *
 *  Created on: Feb 10, 2026
 *      Author: kemazhu
 */
#include "ebyte_Task.h"
#include "stm32f4xx.h"
#include "shared_data.h"
#include "ebyte.h"
#include "FreeRTOS.h"
#include <stdio.h>

void vTaskEBYTE(void *pvParameters)
{
    uint8_t tx_buffer[27]; // 27 octets : capteurs + GPS

    ebyte_init(); // Initialisation USART1

    while (1)
    {
        // === LECTURE DES CAPTEURS + GPS ===
        float temp_int, temp_ext, humidity, sound, accel_x, accel_y, accel_z;
        float latitude, longitude, weight_kg;
        uint8_t water_level, gps_valid;

        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Capteurs environnementaux
            temp_int = g_temperature;
            humidity = g_humidity;
            temp_ext = g_temperature2;
            sound = g_bee_tone_energy;
            water_level = (uint8_t)g_distance_cm;

            // Accéléromètre
            accel_x = g_acc_x;
            accel_y = g_acc_y;
            accel_z = g_acc_z;

            // GPS
            latitude = g_latitude;
            longitude = g_longitude;
            gps_valid = g_gps_valid ? 1 : 0;

            //weight
            weight_kg = g_weight_kg;

           xSemaphoreGive(xSensorMutex);
       }

        // === CONSTRUCTION DE LA TRAME (24 octets) ===
        tx_buffer[0] = 0xAA;
        tx_buffer[1] = HIVE_ID;
        // Températures (×100 → int16_t)
        int16_t t_int = (int16_t)(temp_int * 100.0f);
        tx_buffer[2] = (uint8_t)(t_int >> 8);
        tx_buffer[3] = (uint8_t)(t_int & 0xFF);

        int16_t t_ext = (int16_t)(temp_ext * 100.0f);
        tx_buffer[4] = (uint8_t)(t_ext >> 8);
        tx_buffer[5] = (uint8_t)(t_ext & 0xFF);

        // Humidité + Son + Eau
        tx_buffer[6] = (uint8_t)humidity;
        uint16_t snd = (uint16_t)(sound * 1000.0f);
        tx_buffer[7] = (uint8_t)(snd >> 8);
        tx_buffer[8] = (uint8_t)(snd & 0xFF);
        tx_buffer[9] = water_level;

        // Accéléromètre (×1000 → int16_t)
        int16_t ax = (int16_t)(accel_x * 1000.0f);
        tx_buffer[10] = (uint8_t)(ax >> 8);
        tx_buffer[11] = (uint8_t)(ax & 0xFF);

        int16_t ay = (int16_t)(accel_y * 1000.0f);
        tx_buffer[12] = (uint8_t)(ay >> 8);
        tx_buffer[13] = (uint8_t)(ay & 0xFF);

        int16_t az = (int16_t)(accel_z * 1000.0f);
        tx_buffer[14] = (uint8_t)(az >> 8);
        tx_buffer[15] = (uint8_t)(az & 0xFF);

        // GPS : Latitude ×10⁶ → int32_t
        int32_t lat = (int32_t)(latitude * 1000000.0f);
        tx_buffer[16] = (uint8_t)(lat >> 24);
        tx_buffer[17] = (uint8_t)(lat >> 16);
        tx_buffer[18] = (uint8_t)(lat >> 8);
        tx_buffer[19] = (uint8_t)(lat & 0xFF);

        // GPS : Longitude ×10⁶ → int32_t
        int32_t lon = (int32_t)(longitude * 1000000.0f);
        tx_buffer[20] = (uint8_t)(lon >> 24);
        tx_buffer[21] = (uint8_t)(lon >> 16);
        tx_buffer[22] = (uint8_t)(lon >> 8);
        tx_buffer[23] = (uint8_t)(lon & 0xFF);

        // Statut (bit 0: GPS valide, bit 1-7: réservé)
        tx_buffer[24] = gps_valid;

        // hive_weight
        uint16_t hive_weight = (uint16_t)(weight_kg * 100.0f);
        tx_buffer[25] = (uint8_t)(hive_weight >> 8);
        tx_buffer[26] = (uint8_t)(hive_weight & 0xFF);


        // === ENVOI DIRECT VIA USART1 ===
        for (uint16_t i = 0; i < 27; i++) {
            while (!(USART1->SR & USART_SR_TXE));
            USART1->DR = tx_buffer[i];
        }

        // === DEBUG UART2 ===
        printf("TX E220 [%u]: ", HIVE_ID);
        for(int i = 0; i < 27; i++) {
            printf("%02X ", tx_buffer[i]);
        }
        printf("\n");

        // === FEEDBACK LED ===
       // GPIOF->BSRR = (1U << 10);
       // vTaskDelay(pdMS_TO_TICKS(50));
       // GPIOF->BSRR = (1U << (16 + 10));

      //  vTaskDelay(pdMS_TO_TICKS(5000)); // Test: 5s
         vTaskDelay(pdMS_TO_TICKS(2000)); //
    }
}
