/*
 * Delay.h
 *
 *  Created on: Nov 5, 2025
 *      Author: kemazhu
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f4xx.h"

void TimerDelay_Init(void);
void TimerDelay_us(uint32_t us);
void TimerDelay_ms(uint32_t ms);
#endif /* DELAY_H_ */
