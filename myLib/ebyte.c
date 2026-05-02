/*
 * ebyte.c
 *
 *  Created on: Feb 7, 2026
 *      Author: kemazhu
 *      EBYTE E220-400T30D 30dBm LoRa Wireless Module
 */

#include "stm32f4xx.h"
#include "ebyte.h"
#include <stdlib.h>
#include  "delay1.h"

#include "ebyte.h"
#include "FreeRTOS.h"
#include "task.h"

// BRR pour 9600 @ PCLK2 = 84 MHz
#define USART1_BRR_VALUE 0x222E //0x22C

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // PB6 = TX (AF7)
    GPIOB->MODER = (GPIOB->MODER & ~(3U << 12)) | (2U << 12);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFU << 24)) | (7U << 24);

    // PB7 = RX (AF7)
    GPIOB->MODER = (GPIOB->MODER & ~(3U << 14)) | (2U << 14);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFU << 28)) | (7U << 28);

    // PB9 = AUX (input)
    GPIOB->MODER &= ~(3U << 18);
}

static void usart1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = USART1_BRR_VALUE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void ebyte_init(void)
{
    gpio_init();
    usart1_init();
}

static bool wait_aux_ready(void)
{
    TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(10); // 10 ms max

    while ((GPIOB->IDR & (1U << E22_AUX_PIN)) == 0) {
        if ((xTaskGetTickCount() - start) > timeout) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

bool ebyte_transmit(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0 || len > E22_MAX_PACKET_SIZE) {
        return false;
    }

    // Pré-réveil (2-3 ms)
    vTaskDelay(pdMS_TO_TICKS(2));

    // Attendre que le module soit prêt
    if (!wait_aux_ready()) {
        return false;
    }

    // Envoyer les données
    for (uint16_t i = 0; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = data[i];
    }

    return true;
}
