/*
 * ebyte.h
 *
 *  Created on: Feb 7, 2026
 *      Author: kemazhu
 */

#ifndef EBYTE_H_
#define EBYTE_H_


#include <stdint.h>
#include "stm32f4xx.h"
#include <stdbool.h>
// Pins E220
#define E22_TX_PORT   GPIOB
#define E22_TX_PIN    6       // PB6 = USART1_TX
#define E22_RX_PORT   GPIOB
#define E22_RX_PIN    7       // PB7 = USART1_RX
#define E22_AUX_PORT  GPIOB
#define E22_AUX_PIN   9       // PB9 = AUX

// Taille max trame
#define E22_MAX_PACKET_SIZE 26

// Fonctions publiques
void ebyte_init(void);
bool ebyte_transmit(const uint8_t *data, uint16_t len);
#endif /* EBYTE_H_ */

