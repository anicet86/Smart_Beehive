/*
 * inmp411.c
 *
 *  Created on: 14 апр. 2026 г.
 *      Author: kemazhu
 */


#include "inmp441.h"
#include "stm32f4xx.h"
#include <math.h>


// Définition des variables globales
volatile uint16_t rx_buffer[INMP441_BUFFER_SIZE];
volatile uint8_t  sample_ready = 0;

/**
 * @brief Traite un échantillon I2S 16-bit
 * @note Contient la logique de découpage/reconstruction demandée
 */
void INMP441_ProcessSample(uint16_t sample) {
    // Découpage 16-bit -> 2 x 8-bit
    uint8_t byte_low  = sample & 0xFF;
    uint8_t byte_high = (sample >> 8) & 0xFF;

    // Reconstruction 16-bit (identique à ton exemple HAL)
    uint16_t reconstructed = ((uint16_t)byte_high << 8) | byte_low;

    // TODO: Ajoute ici ton calcul RMS, envoi UART, ou stockage dans g_bee_tone_energy
    // Exemple: g_bee_tone_energy += (float)reconstructed;
    (void)reconstructed; // Évite le warning "unused variable"
}

/**
 * @brief Initialisation complète I2S2 + DMA1 Stream4 pour INMP441
 */
void INMP441_Init(void) {
    // 1. Horloges
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // 2. PLLI2S pour générer l'horloge I2S (~48 kHz sample rate)
    RCC->PLLI2SCFGR = (258 << 6) | (3 << 28);
    RCC->CR |= RCC_CR_PLLI2SON;

    uint32_t timeout = 0x200000;
    while (timeout-- && !(RCC->CR & RCC_CR_PLLI2SRDY));
    if (timeout == 0) while(1); // Bloque si PLLI2S échoue (debug)

    // 3. GPIO: PB12(WS), PB13(CK), PB15(SD) -> AF5
    GPIOB->MODER &= ~(0x3U<<24 | 0x3U<<26 | 0x3U<<30);
    GPIOB->MODER |=  (0x2U<<24 | 0x2U<<26 | 0x2U<<30); // Alternate Function
    GPIOB->OSPEEDR |= (0x3U<<24 | 0x3U<<26 | 0x3U<<30); // High Speed
    GPIOB->AFR[1] &= ~(0xFU<<16 | 0xFU<<20 | 0xFU<<28);
    GPIOB->AFR[1] |=  (5U<<16 | 5U<<20 | 5U<<28);       // AF5 = SPI2/I2S2

    // 4. Configuration I2S2: Master Rx, Philips, 16-bit data, 16-bit frame
    SPI2->I2SCFGR = 0;
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD;                    // Mode I2S
    SPI2->I2SCFGR |= (3 << SPI_I2SCFGR_I2SCFG_Pos);         // Master Receive (11)
    SPI2->I2SCFGR |= (0 << SPI_I2SCFGR_I2SSTD_Pos);         // Philips Standard
    SPI2->I2SCFGR |= (0 << SPI_I2SCFGR_DATLEN_Pos);         // 16-bit Data
    SPI2->I2SCFGR |= (0 << SPI_I2SCFGR_CHLEN_Pos);          // 16-bit Frame
    SPI2->I2SPR = 27;                                       // Prescaler ~48kHz
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;                      // Activer I2S

    // 5. Configuration DMA1 Stream4 (SPI2_RX)
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN); // Attendre arrêt complet

    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)rx_buffer;
    DMA1_Stream4->NDTR = INMP441_BUFFER_SIZE;

    DMA1_Stream4->CR =
        (2U << DMA_SxCR_CHSEL_Pos) | // Channel 2 for SPI2_RX
        DMA_SxCR_MINC |              // Incrémenter adresse mémoire
        DMA_SxCR_PSIZE_0 |           // Taille périphérique: 16-bit
        DMA_SxCR_MSIZE_0 |           // Taille mémoire: 16-bit
        (0U << DMA_SxCR_DIR_Pos) |   // Direction: Peripheral -> Memory
        DMA_SxCR_CIRC |              // ⭐ Mode circulaire (auto-reboucle)
        DMA_SxCR_HTIE | DMA_SxCR_TCIE; // Interruptions Half & Transfer Complete

    // 6. NVIC
    NVIC_SetPriority(DMA1_Stream4_IRQn, 5); // Priorité compatible FreeRTOS
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    // 7. Démarrage DMA + I2S
    SPI2->CR2 |= SPI_CR2_RXDMAEN;
    DMA1_Stream4->CR |= DMA_SxCR_EN;

    // 8. Attendre le démarrage hardware de l'INMP441 (85ms requis par la datasheet)
    for(volatile int i=0; i<1500000; i++);
}

/**
 * @brief Gestionnaire d'interruption DMA1 Stream4
 * @note Appelée automatiquement par le vecteur d'interruption
 */
/*void DMA1_Stream4_IRQHandler(void) {
    // Half-Transfer : première moitié du buffer remplie
    if (DMA1->HISR & DMA_HISR_HTIF4) {
        DMA1->HIFCR = DMA_HIFCR_CHTIF4;
        INMP441_ProcessSample(rx_buffer[0]);
        sample_ready = 1;
    }
    // Transfer-Complete : seconde moitié du buffer remplie
    if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4;
        INMP441_ProcessSample(rx_buffer[1]);
        sample_ready = 1;
    }
}*/
