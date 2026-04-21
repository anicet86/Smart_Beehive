/*
 * hc_sr04.h
 *
 *  Created on: Jan 24, 2026
 *      Author: kemazhu
 */
#ifndef HC_SR04_H_
#define HC_SR04_H_

#include <stdint.h>
#include "stm32f4xx.h"

// Broches : PA6 = Trig, PA7 = Echo
#define HCSR04_TRIG_PORT  GPIOA
#define HCSR04_TRIG_PIN   6
#define HCSR04_ECHO_PORT  GPIOA
#define HCSR04_ECHO_PIN   7
typedef struct {
	uint16_t distance_cm;   /* distance in centimeters */
	float    distance_m;    /* distance in meters      */
    int      status;        /* 0 = OK, 1 = timeout */
} HCSR04_Data;

void HCSR04_Init(void);
uint32_t HCSR04_Read(HCSR04_Data *data);

#endif /* HC_SR04_H_ */
