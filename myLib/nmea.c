/*
 * nmea.c
 *
 *  Created on: Jan 27, 2026
 *      Author: kemazhu
 */

#include "stm32f4xx.h"
#include "nmea.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>



// Variables globales
int GMT = +530;
int inx = 0;
int hr = 0, min = 0, day = 0, mon = 0, yr = 0;
int daychange = 0;

char nmea_buffer[NMEA_BUFFER_SIZE] = {0};
volatile uint8_t gpgga_ready = 0;
GGASTRUCT gga_data = {0};

static uint8_t idx = 0;
static uint8_t in_line = 0;

void nmea_init(void)
{
    gpgga_ready = 0;
    idx = 0;
    in_line = 0;
    memset(&gga_data, 0, sizeof(GGASTRUCT));
}

void nmea_parse(uint8_t c)
{
    if (c == '$') {
        in_line = 1;
        idx = 0;
        nmea_buffer[idx++] = c;
        return;
    }

    if (!in_line) return;

    if (c == '\r' || c == '\n') {
        nmea_buffer[idx] = '\0';

        //  Traiter UNIQUEMENT $GPGGA
        if (strncmp(nmea_buffer, "$GPGGA", 6) == 0) {
            if (decodeGGA(nmea_buffer, &gga_data) == 0) {
                gpgga_ready = 1; // Trame valide décodée
            }
        }

        in_line = 0;
        idx = 0;
        return;
    }

    if (idx < NMEA_BUFFER_SIZE - 1) {
        nmea_buffer[idx++] = c;
    } else {
        in_line = 0;
    }
}

// fonction decodeGGA
int decodeGGA(char *GGAbuffer, GGASTRUCT *gga)
{
    inx = 0;
    char buffer[12];
    int i = 0;
    while (GGAbuffer[inx] != ',') inx++;  // 1st ','
    inx++;
    while (GGAbuffer[inx] != ',') inx++;  // After time ','
    inx++;
    while (GGAbuffer[inx] != ',') inx++;  // after latitude ','
    inx++;
    while (GGAbuffer[inx] != ',') inx++;  // after NS ','
    inx++;
    while (GGAbuffer[inx] != ',') inx++;  // after longitude ','
    inx++;
    while (GGAbuffer[inx] != ',') inx++;  // after EW ','
    inx++;  // reached the character to identify the fix
    if ((GGAbuffer[inx] == '1') || (GGAbuffer[inx] == '2') || (GGAbuffer[inx] == '6'))   // 0 indicates no fix yet
    {
        gga->isfixValid = 1;   // fix available
        inx = 0;   // reset the index. We will start from the inx=0 and extract information now
    }
    else
    {
        gga->isfixValid = 0;   // If the fix is not available
        return 1;  // return error
    }
    while (GGAbuffer[inx] != ',') inx++;  // 1st ','


    /*********************** Get TIME ***************************/
    //(Update the GMT Offset at the top of this file)

    inx++;   // reach the first number in time
    memset(buffer, 0, 12);
    i=0;
    while (GGAbuffer[inx] != ',')  // copy upto the we reach the after time ','
    {
        buffer[i] = GGAbuffer[inx];
        i++;
        inx++;
    }

    hr = (atoi(buffer)/10000) + GMT/100;   // get the hours from the 6 digit number

    min = ((atoi(buffer)/100)%100) + GMT%100;  // get the minutes from the 6 digit number

    // adjust time.. This part still needs to be tested
    if (min > 59)
    {
        min = min-60;
        hr++;
    }
    if (hr<0)
    {
        hr=24+hr;
        daychange--;
    }
    if (hr>=24)
    {
        hr=hr-24;
        daychange++;
    }

    // Store the time in the GGA structure
    gga->tim.hour = hr;
    gga->tim.min = min;
    gga->tim.sec = atoi(buffer)%100;

    /***************** Get LATITUDE  **********************/
    inx++;   // Reach the first number in the lattitude
    memset(buffer, '\0', 12);
    i=0;
    while (GGAbuffer[inx] != ',')   // copy upto the we reach the after lattitude ','
    {
        buffer[i] = GGAbuffer[inx];
        i++;
        inx++;
    }
    if (strlen(buffer) < 6) return 2;  // If the buffer length is not appropriate, return error
    int16_t num = (atoi(buffer));   // change the buffer to the number. It will only convert upto decimal
    int j = 0;
    while (buffer[j] != '.') j++;   // Figure out how many digits before the decimal
    j++;
    int declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
    int dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
    float lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
    gga->lcation.latitude = lat;  // save the lattitude data into the strucure
    inx++;
    gga->lcation.NS = GGAbuffer[inx];  // save the N/S into the structure


    /***********************  GET LONGITUDE **********************/
    inx++;  // ',' after NS character
    inx++;  // Reach the first number in the longitude
    memset(buffer, '\0', 12);
    i=0;
    while (GGAbuffer[inx] != ',')  // copy upto the we reach the after longitude ','
    {
        buffer[i] = GGAbuffer[inx];
        i++;
        inx++;
    }
    num = (atoi(buffer));  // change the buffer to the number. It will only convert upto decimal
    j = 0;
    while (buffer[j] != '.') j++;  // Figure out how many digits before the decimal
    j++;
    declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
    dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
    lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
    gga->lcation.longitude = lat;  // save the longitude data into the strucure
    inx++;
    gga->lcation.EW = GGAbuffer[inx];  // save the E/W into the structure

    /**************************************************/
    // skip positition fix
    inx++;   // ',' after E/W
    inx++;   // position fix
    inx++;   // ',' after position fix;

    // number of sattelites
    inx++;  // Reach the first number in the satellites
    memset(buffer, '\0', 12);
    i=0;
    while (GGAbuffer[inx] != ',')  // copy upto the ',' after number of satellites
    {
        buffer[i] = GGAbuffer[inx];
        i++;
        inx++;
    }
    gga->numofsat = atoi(buffer);   // convert the buffer to number and save into the structure


    /***************** skip HDOP  *********************/
    inx++;
    while (GGAbuffer[inx] != ',') inx++;


    /*************** Altitude calculation ********************/
    inx++;
    memset(buffer, '\0', 12);
    i=0;
    while (GGAbuffer[inx] != ',')
    {
        buffer[i] = GGAbuffer[inx];
        i++;
        inx++;
    }
    num = (atoi(buffer));
    j = 0;
    while (buffer[j] != '.') j++;
    j++;
    declen = (strlen(buffer))-j;
    dec = atoi ((char *) buffer+j);
    lat = (num) + (dec/pow(10, (declen)));
    gga->alt.altitude = lat;

    inx++;
    gga->alt.unit = GGAbuffer[inx];

    return 0;
}
