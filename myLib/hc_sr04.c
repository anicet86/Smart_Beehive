/*
 * hc_sr04.c
 *
 *  Created on: Jan 24, 2026
 *      Author: kemazhu
 */

#include "hc_sr04.h"
#include "stm32f4xx.h"
#include "Delay.h"


#include "hc_sr04.h"
#include "stm32f4xx.h"
#include "Delay.h" // Ton fichier de délai existant

// Assure-toi que SystemCoreClock = 16000000
extern uint32_t SystemCoreClock;

/* Active DWT pour mesures précises */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Configuration GPIO */
static void GPIOA_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA6 = TRIG (sortie push-pull)
    GPIOA->MODER &= ~(3U << (HCSR04_TRIG_PIN * 2));
    GPIOA->MODER |= (1U << (HCSR04_TRIG_PIN * 2));
    GPIOA->OTYPER &= ~(1U << HCSR04_TRIG_PIN);
    GPIOA->OSPEEDR |= (2U << (HCSR04_TRIG_PIN * 2));
    GPIOA->PUPDR &= ~(3U << (HCSR04_TRIG_PIN * 2));

    // PA7 = ECHO (entrée FLOTTANTE - pas de pull !)
    GPIOA->MODER &= ~(3U << (HCSR04_ECHO_PIN * 2));
    GPIOA->PUPDR &= ~(3U << (HCSR04_ECHO_PIN * 2)); //  CRITIQUE
}

/* Fonctions inline pour Trig/Echo */
static inline void trig_set_high(void)
{
    HCSR04_TRIG_PORT->BSRR = (1U << HCSR04_TRIG_PIN);
}

static inline void trig_set_low(void)
{
    HCSR04_TRIG_PORT->BSRR = (1U << (HCSR04_TRIG_PIN + 16U));
}

static inline uint32_t echo_read(void)
{
    return (HCSR04_ECHO_PORT->IDR >> HCSR04_ECHO_PIN) & 0x1U;
}

/* Initialisation */
void HCSR04_Init(void)
{
    // SystemCoreClock doit être 16000000 (HSI)
    DWT_Init();
    GPIOA_Init();
    trig_set_high();
    trig_set_low();
}

/* Mesure du pulse Echo */
static uint32_t measure_echo_pulse_us(void)
{
    // Timeout : 25 ms max (4 mètres)
    const uint32_t timeout_cycles = 25000 * (SystemCoreClock / 1000000); // 25000 * 16 = 400000 cycles

    // Attendre front montant
    uint32_t start_wait = DWT->CYCCNT;
    while (!echo_read()) {
        if ((DWT->CYCCNT - start_wait) >= timeout_cycles) {
            return 0; // Timeout
        }
    }

    // Mesurer durée du pulse
    uint32_t start = DWT->CYCCNT;
    while (echo_read());
    uint32_t end = DWT->CYCCNT;

    // Convertir en microsecondes
    return (end - start) / (SystemCoreClock / 1000000); // /16 pour 16 MHz
}

/* Lecture principale */
uint32_t HCSR04_Read(HCSR04_Data *data)
{
    if (data == NULL) {
        return 1;
    }

    // Impulsion Trig (10 µs)
    trig_set_high();
    TimerDelay_us(10); // délai existant (doit fonctionner à 16 MHz)
    trig_set_low();

    // Mesurer le pulse Echo
    uint32_t pulse_us = measure_echo_pulse_us();
    if (pulse_us == 0 || pulse_us > 24000) { // Max ~4.1 m
        data->status = 1;
        data->distance_cm = 0;
        data->distance_m = 0.0f;
        return 1;
    }

    // Calcul de la distance
    data->distance_cm = (uint16_t)(pulse_us / 58U);
    data->distance_m = data->distance_cm / 100.0f;
    data->status = 0;

    return 0;
}
