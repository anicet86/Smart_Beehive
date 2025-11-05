/*
 * dht22.h
 *
 *  Created on: Nov 4, 2025
 *      Author: kemazhu
 */

#ifndef DHT22_H_
#define DHT22_H_

#include "stm32f4xx.h"
#include "delay.h"

#define DHT22_GPIO_PORT GPIOA
#define DHT22_PIN       1

typedef struct {
    float temperature;
    float humidity;
} DHT22_Data;

void DHT22_Init(void);
uint8_t DHT22_Read(DHT22_Data *data);
#endif /* DHT22_H_ */
