/*
 * adxl335.h
 *
 *  Created on: Dec 1, 2025
 *      Author: kemazhu
 */

#ifndef ADXL335_H_
#define ADXL335_H_


#include <stdint.h>
typedef struct{
float accx;
float accy;
float accz;
}acc_data_t;


void adxl335_Init(void);// функция инициализации регистров GPIOA и ADC
void adxl335_Read(acc_data_t *acc);//функция считывания оси ускорения датчика (X,Y,Z)

#endif /* ADXL335_H_ */
