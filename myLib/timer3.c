/*
 * pwm.c
 *
 *  Created on: Nov 11, 2025
 *      Author: kemazhu
 */

#include "stm32f407xx.h"
#include "dht22.h"
#include "pid.h"
#include "timer3.h"
/******************************************************************************************************
 *  la fonction PWM sera utilise pour generer les temps de reactiondu  regulateur PID pour la commande
 *  du ventillateur et de l'element de chauffage les broches PB1 et PB0 seront utilise pour connecter
 *   l'element de chauffage et le ventillateur
 *****************************************************************************************************/



// Fréquence PWM : 1 kHz → période = 1 ms
// APB1 = 84 MHz (typique), prédiviseur = 84 → timer clock = 1 MHz → ARR = 999 → 1 kHz
#define PWM_FREQUENCY_HZ 1000
#define TIMER_CLK_HZ     84000000  // APB1 = 84 MHz (STM32F407 default)
#define PRESCALER        84        // 84 MHz / 84 = 1 MHz
#define ARR_VALUE        (1000 - 1) // 1 MHz / 1000 Hz = 1000 ticks → 0 à 999

void PWM_Init(void)
{
    // 1. Activer horloges
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 2. Configurer PB0 (TIM3_CH3) et PB1 (TIM3_CH4) en mode alternate function
    GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1; // Alternate function mode
    GPIOB->AFR[0] |= (0x02 << (0 * 4)) | (0x02 << (1 * 4));   // AF2 = TIM3 sur PB0/PB1

    // 3. Configurer TIM3
    TIM3->PSC = PRESCALER - 1;  // 84 - 1 → division par 84
    TIM3->ARR = ARR_VALUE;      // 999 → période = (999+1)/1MHz = 1 ms → 1 kHz

    // PWM mode 1: OCx active high, duty = CCRx / (ARR+1)
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 |  // PWM mode 1 pour CH3
                   TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;   // PWM mode 1 pour CH4

    TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE; // Preload enable

    // 4. Activer les canaux
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;

    // 5. Démarrer le timer
    TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void PWM_SetHeater(float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t ccr = (uint32_t)((duty_percent / 100.0f) * (ARR_VALUE + 1));
    TIM3->CCR4 = ccr; // PB1 = TIM3_CH4
}

void PWM_SetFan(float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t ccr = (uint32_t)((duty_percent / 100.0f) * (ARR_VALUE + 1));
    TIM3->CCR3 = ccr; // PB0 = TIM3_CH3
}
