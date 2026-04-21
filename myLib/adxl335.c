
/*
 * adxl335.c
 *
 *  Created on: Dec 3, 2025
 *      Author: kemazhu
 */

#include "adxl335.h"
#include "stm32f407xx.h"


#define ADC_CH_X 10U  // PC0 = ADC1_IN10 OUT_X
#define ADC_CH_Y 11U  // PC1 = ADC1_IN11 OUT_Y
#define ADC_CH_Z 12U  // PC2 = ADC1_IN12 OUT_7

static float adc_to_gx(uint16_t val) //функция преобразования напряжения в ускорении
{
    float v = (val * 3.3f) / 4095.0f;
    return (v - 1.65f) / 0.365f;
}

static float adc_to_gy(uint16_t val) //функция преобразования напряжения в ускорении
{
    float v = (val * 3.3f) / 4095.0f;
    return (v - 1.65f) / 0.355f;
}
static float adc_to_gz(uint16_t val) //функция преобразования напряжения в ускорении
{
    float v = (val * 3.3f) / 4095.0f;
    return (v - 1.65f) / 0.41f;
}

void adxl335_Init(void)// функция инициализации регистров GPIOA и ADC
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //конфигурация часов GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;//конфигурация часов ADC1

    GPIOC->MODER |= GPIO_MODER_MODER0 // конфигурация выводов в аналоговом режиме
                  | GPIO_MODER_MODER1
                  | GPIO_MODER_MODER2;

    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT;

    ADC1->SQR1 = (2U << 20); //
    ADC1->SQR3|=(ADC_CH_X<<ADC_SQR3_SQ1_Pos)|(ADC_CH_Y<<ADC_SQR3_SQ2_Pos)|(ADC_CH_Z<<ADC_SQR3_SQ3_Pos);
   /* ADC1->SQR3 = (ADC_CH_X << 0)// Rang 1 : X (PC0)
               | (ADC_CH_Y << 5)// Rang 2 : Y (PC1)
               | (ADC_CH_Z << 10);// Rang 3 : Z (PC2)*/

    // Temps d'échantillonnage : 480 cycles pour canaux 10,11,12//Время отбора проб: 480 циклов для каналов 10,11,12
        // Canaux 10-12 → SMPR1 (bits 0-8)
    ADC1->SMPR1 = (7U << 0) | (7U << 3) | (7U << 6);

    for (volatile int i = 0; i < 100000; i++); // Stabilisation
}


void adxl335_Read(acc_data_t *acc) //функция считывания оси ускорения датчика (X,Y,Z)
{
    ADC1->CR2 |= ADC_CR2_SWSTART; // Lancer conversion //Начать преобразование



    while (!(ADC1->SR &  ADC_SR_EOC)); //first conversion = X (PC0 ADC3_IN10)
    acc->accx = adc_to_gx(ADC1->DR);
    while (!(ADC1->SR & ADC_SR_EOC));// thirt conversion = Z  (PC2 ADC_IN12)
    acc->accy = adc_to_gy(ADC1->DR);
    while (!(ADC1->SR & ADC_SR_EOC)); //second conversion = Y (PC1 ADC3_IN11)
    acc->accz = adc_to_gz(ADC1->DR);
}
