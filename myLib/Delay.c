/*
 * Delay.c
 *
 *  Created on: Nov 5, 2025
 *      Author: kemazhu
 */


#include "delay.h"

void TimerDelay_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // Active l'horloge TIM2
    TIM2->PSC = (SystemCoreClock / 1000000) - 1; // 1 µs par tick
    TIM2->ARR = 0xFFFFFFFF;                 // compteur max 32 bits
    TIM2->EGR = TIM_EGR_UG;                // maj des registres
    TIM2->CR1 |= TIM_CR1_CEN;              // démarre le timer
}

void TimerDelay_us(uint32_t us) // fonction pour compter en microseconde
{
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
}

void TimerDelay_ms(uint32_t ms)// fonction pour compter en millisecode
{
    while (ms--)
        TimerDelay_us(1000);
}
