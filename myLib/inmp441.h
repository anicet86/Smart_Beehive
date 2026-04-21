/*
 * INMP411.h
 *
 *  Created on: 14 апр. 2026 г.
 *      Author: kemazhu
 */

#ifndef INMP441_H_
#define INMP441_H_
#include <stdint.h>

#include <stdint.h>

// Taille du buffer DMA (2 échantillons : Half-Transfer + Transfer-Complete)
#define INMP441_BUFFER_SIZE 2

// Variables globales (accessibles depuis main.c ou autres tâches)
extern volatile uint16_t rx_buffer[INMP441_BUFFER_SIZE];
extern volatile uint8_t  sample_ready;

// Prototypes des fonctions publiques
void INMP441_Init(void);
void INMP441_ProcessSample(uint16_t sample);
#endif /* INMP441_H_ */
