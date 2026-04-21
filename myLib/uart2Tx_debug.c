/*
 * uartTx_debug.c
 *
 *  Created on: Jan 6, 2026
 *      Author: kemazhu
 */
#include "stm32f407xx.h"
#include <stdio.h>

void uart2_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2 = USART2_TX (AF7)
    GPIOA->MODER |= GPIO_MODER_MODER2_1;  //PA2 TX
    GPIOA->AFR[0] |= (7U << 8);

    // PA3 = RX (AF7)
       GPIOA->MODER = (GPIOA->MODER & ~(3U << 6)) | (2U << 6);
       GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << 12)) | (7U << 12);

    // 115200 bauds, 8N1 (PCLK1 = 42 MHz)
    USART2->BRR = 42000000 / 115200; // = 364 → 0x16C
    USART2->CR1 = USART_CR1_TE| USART_CR1_RE| USART_CR1_UE; // TX enable + USART enable
}

// Fonction printf vers UART
int _write(int fd, char* ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *ptr++;
    }
    return len;
}


