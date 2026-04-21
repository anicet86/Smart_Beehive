/*
 * DS18B20.h
 *
 *  Created on: Nov 22, 2025
 *      Author: kemazhu
 */

#ifndef DS18B20_H_
#define DS18B20_H_
#include <stdint.h>
//#include "Delay.h"
// Structure pour stocker les données du capteur
typedef struct {
    float temperature;      // Température en °C
    uint8_t rom[8];         // ROM unique du capteur (64-bit)
    uint8_t last_status;    // 0 = OK, autre = erreur
} DS18B20_Data;

// Codes d'erreur
#define DS18B20_OK          0
#define DS18B20_CRC_ERROR   1
#define DS18B20_NO_DEVICE   2
#define DS18B20_TIMEOUT     3
// Fonctions publiques
void DS18B20_Init(void);
uint8_t DS18B20_ReadTemperature(DS18B20_Data *data);



#endif /* DS18B20_H_ */
