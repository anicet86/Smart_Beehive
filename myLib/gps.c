/*
 * nme.c
 *
 *  Created on: Dec 8, 2025
 *      Author: kemazhu
 */
#include "gps.h"
#include "stm32f4xx.h"
#include "shared_data.h"
#include "nmea.h"



static volatile uint8_t gps_buffer[GPS_BUFFER_SIZE];
static volatile uint32_t read_index = 0;



void (*gps_rx_callback)(uint8_t) = NULL;

void gps_SetCallback(void (*callback)(uint8_t))
{
    gps_rx_callback = callback;
}

static void Gpio_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(3U << 22);      // PB11
    GPIOB->MODER |= (2U << 22);       // AF
    GPIOB->AFR[1] &= ~(0xFU << 12);
    GPIOB->AFR[1] |= (7U << 12);     // AF7
    GPIOB->PUPDR &= ~(3U << 22);      // No pull
}

static void Usart3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    USART3->CR1 = 0; // disable USART1

    // 9600 bauds @ 42MHz
    USART3->BRR = 0x1117;

    // Activer RX interrupt + receiver
    USART3->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    // NVIC
    NVIC_SetPriority(USART3_IRQn, 2);
    NVIC_EnableIRQ(USART3_IRQn);
}

void Gps_Init(void)
{
    Gpio_Init();
    Usart3_Init();
}

// 🔥 Nouvelle ISR : USART3, pas DMA !
void USART3_IRQHandler(void)
{
    if (USART3->SR & USART_SR_RXNE) {
        uint8_t c = USART3->DR; // Lire l'octet

        if (gps_rx_callback) {
            gps_rx_callback(c);
        }
    }
}
