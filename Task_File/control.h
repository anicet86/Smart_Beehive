/*
 * control.h
 *
 *  Created on: Nov 30, 2025
 *      Author: kemazhu
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "FreeRTOS.h"
#include "task.h"

void vTaskControl(void *pvParameters); // control temperatur and humidity
//void vTaskControl1(void *pvParameters); // control inclinaison
#endif /* CONTROL_H_ */
