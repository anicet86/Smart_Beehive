/*
 * nmea.h
 *
 *  Created on: Dec 8, 2025
 *      Author: kemazhu
 */

#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include "stm32f4xx.h"

#define GPS_BUFFER_SIZE 256

extern void (*gps_rx_callback)(uint8_t);
void Gps_Init(void);
void gps_SetCallback(void (*callback)(uint8_t));

#endif /* NMEA_H_ */
