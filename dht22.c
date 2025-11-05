/*
 * dht22.c
 *
 *  Created on: Nov 4, 2025
 *      Author: kemazhu
 */

#include "dht22.h"
#include "stm32f407xx.h"
#include <string.h>

#include "dht22.h"

/* Fonctions de manipulation du GPIO mode output */
static void DHT22_SetPinOutput(void)
{
    DHT22_GPIO_PORT->MODER &= ~(3U << (DHT22_PIN * 2)); //ici on reset le pin PA2 
    DHT22_GPIO_PORT->MODER |=  (1U << (DHT22_PIN * 2)); // configuration Output
    DHT22_GPIO_PORT->OTYPER &= ~(1U << DHT22_PIN);       // mode Push-pull
    DHT22_GPIO_PORT->OSPEEDR |= (3U << (DHT22_PIN * 2)); // High speed
}
/* Fonctions de manipulation du GPIO mode input */
static void DHT22_SetPinInput(void)
{
    DHT22_GPIO_PORT->MODER &= ~(3U << (DHT22_PIN * 2)); // Input
    DHT22_GPIO_PORT->PUPDR &= ~(3U << (DHT22_PIN * 2)); // Pas de pull
}

static void DHT22_WriteBit(uint8_t value)
{
    if (value)
        DHT22_GPIO_PORT->BSRR = (1U << DHT22_PIN);
    else
        DHT22_GPIO_PORT->BSRR = (1U << (DHT22_PIN + 16));
}

static uint8_t DHT22_ReadPin(void)
{
    return (DHT22_GPIO_PORT->IDR & (1U << DHT22_PIN)) != 0;
}

/* Initialisation du GPIO */
void DHT22_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    DHT22_SetPinInput();
}

/* Lecture complète */
uint8_t DHT22_Read(DHT22_Data *data)
{
    uint8_t bits[5] = {0};
    uint32_t timeout;

    /* Start signal */
    DHT22_SetPinOutput();
    DHT22_WriteBit(0);
    TimerDelay_ms(2);       // minimum 1ms
    DHT22_WriteBit(1);
    TimerDelay_us(40);
    DHT22_SetPinInput();

    /* Attente de la réponse du capteur */
    timeout = 0;
    while (DHT22_ReadPin() && timeout++ < 10000);
    while (!DHT22_ReadPin() && timeout++ < 10000);
    while (DHT22_ReadPin() && timeout++ < 10000);

    /* Lecture des 40 bits */
    for (int i = 0; i < 40; i++)
    {
        while (!DHT22_ReadPin());          // attente front montant
        TimerDelay_us(30);
        if (DHT22_ReadPin())               // si encore haut après 30 µs => ‘1’
            bits[i / 8] |= (1 << (7 - (i % 8)));
        while (DHT22_ReadPin());
    }

    /* Checksum */
    if (((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF) != bits[4]) // ici on verifi le bit de parite 
        return 1; // erreur

    uint16_t rawHumidity = (bits[0] << 8) | bits[1];
    uint16_t rawTemp = (bits[2] << 8) | bits[3];

    data->humidity = rawHumidity / 10.0f;
    if (rawTemp & 0x8000)
        data->temperature = -((rawTemp & 0x7FFF) / 10.0f); // ici on convertis la temperature mesuree en valeur reel
    else
        data->temperature = rawTemp / 10.0f;

    return 0;
}
