/*
 * nmea.h
 *
 *  Created on: Jan 27, 2026
 *      Author: kemazhu
 */

#ifndef NMEA_H_
#define NMEA_H_


#include <stdint.h>

#define NMEA_BUFFER_SIZE 80

// Structure GGA
typedef struct {
    float latitude;
    float longitude;
    char NS;  // 'N' ou 'S'
    char EW;  // 'E' ou 'W'
} GgaLocation;

typedef struct {
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} GgaTime;

typedef struct {
    float altitude;
    char unit;
} GgaAltitude;

typedef struct {
    GgaTime tim;
    GgaLocation lcation;
    GgaAltitude alt;
    uint8_t numofsat;
    uint8_t isfixValid;
} GGASTRUCT;

// Variables globales
extern char nmea_buffer[NMEA_BUFFER_SIZE];
extern volatile uint8_t gpgga_ready;
extern GGASTRUCT gga_data;

// Fonctions
void nmea_init(void);
void nmea_parse(uint8_t c);
int decodeGGA(char *GGAbuffer, GGASTRUCT *gga);

#endif /* NMEA_H_ */
