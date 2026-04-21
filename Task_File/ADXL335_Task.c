/*
 * ADXL335.c
 *
 *  Created on: Dec 3, 2025
 *      Author: kemazhu
 */

// ADXL335_Task.c
#include "FreeRTOS.h"
#include "task.h"
#include "adxl335.h"
#include "shared_data.h"
#include <stdio.h>
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.14159265359f)//угол в степень преобразования макрос

void vTaskADXL335(void *pvParameters)
{
    acc_data_t acc;
    adxl335_Init();

#define NB_SAMPLES 100
float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;//выборка значений, считанных ADC

    for (int i = 0; i < NB_SAMPLES; i++)
    {
        adxl335_Read(&acc);
        sum_x += acc.accx;
        sum_y += acc.accy;
        sum_z += acc.accz;
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
    float OFFSET_X = sum_x / NB_SAMPLES- 0.1;// Z doit être 1.0g à plat
    float OFFSET_Y = sum_y / NB_SAMPLES-0.042;
    float OFFSET_Z = (sum_z / NB_SAMPLES) - 1.0f; // Z doit être 1.0g à plat




    while (1)
    {
        adxl335_Read(&acc);

		 //**************************Расчет углов*************************//

      float roll = atan2f(g_acc_y,g_acc_z ); //acc.accy acc.accz Поворот (наклон вокруг оси Х)
      float pitch = atan2f(- g_acc_x,
                    sqrtf( g_acc_y* g_acc_y +
                    g_acc_z* g_acc_z));// acc.accx, acc.accy,acc.acczПоворот (наклон вокруг оси y)

		  //***************************************************************//

		  // Conversion en degrés
		          float roll_deg = roll * RAD_TO_DEG;
		          float pitch_deg = pitch * RAD_TO_DEG;


        // Mettre à jour données partagées
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)

        {

              acc.accx -= OFFSET_X;
              acc.accy -= OFFSET_Y;
              acc.accz -= OFFSET_Z; // acc.accy -= OFFSET_Y;
             //acc.accz -= OFFSET_Z
        	  g_acc_x =  acc.accx  ;
              g_acc_y = acc.accy;
              g_acc_z = acc.accz ;//- OFFSET_Z ;
              g_inclinaisonX = roll_deg;
              g_inclinaisonY = pitch_deg;


            xSemaphoreGive(xSensorMutex);
        }

        printf("ADXL335: X=%.1f Y=%.1f Z=%.1f\n", acc.accx,acc.accy, acc.accz);
       vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

