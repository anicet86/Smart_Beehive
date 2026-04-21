#include <shared_data.h>
#include "control.h"
#include "pid.h"
#include "timer3.h"
#include "task.h"
#include "FreeRTOS.h"


#define TARGET_TEMP  (10.0f)   //  температура не должна превышаться 26 °C
#define TARGET_HUMID (5.0f)   // влажность не прошла 30 %


void vTaskControl(void *pvParameters)
{

static PID_Controller_t pid_temp = {  // температурный параметр PID
	    .Kp = 2.0f,
		.Ki = 0.1f,
		.Kd = 0.05f,
	    .setpoint = TARGET_TEMP,
		.prev_error = 0.0f,
		.integral = 0.0f
	};

static PID_Controller_t pid_humid = { //// Параметр влажности PID
    .Kp = 1.5f,
	.Ki = 0.05f,
	.Kd = 0.02f,
    .setpoint = TARGET_HUMID,
	.prev_error = 0.0f,
	.integral = 0.0f
};

float temp_dht22 = 0.0f;
float humid_dht22 = 0.0f;
float out_temp = 0.0f;
float out_humid = 0.0f;

//static PID_Controller_t pid_temp = { /* ... */ };
//static PID_Controller_t pid_humid = { /* ... */ }

	while(1)
	{
		if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(20)) == pdTRUE)
		{
			temp_dht22 = g_temperature;
			 humid_dht22  = g_humidity;
			 xSemaphoreGive(xSensorMutex);
		}

		out_temp = PID_Compute(&pid_temp, temp_dht22);
		out_humid = PID_Compute(&pid_humid, humid_dht22);
		PWM_SetHeater(out_temp);
		PWM_SetFan(out_humid);

        vTaskDelay(pdMS_TO_TICKS(100));
	}

}




/*
 * control.c
 *
 *  Created on: Nov 30, 2025
 *      Author: kemazhu
 */


