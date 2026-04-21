/*
 * pwm.h
 *
 *  Created on: Nov 11, 2025
 *      Author: kemazhu
 */

#ifndef TIMER3_H_
#define TIMER3_H_


void PWM_Init(void);
void PWM_SetHeater(float duty_percent);   // 0.0f à 100.0f
void PWM_SetFan(float duty_percent);      // 0.0f à 100.0f


#endif /* TIMER3_H_ */
