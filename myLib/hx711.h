/*
 * hx711.h
 *
 *  Created on: Mar 14, 2026
 *      Author: kemazhu
 */

#ifndef HX711_H_
#define HX711_H_

#include <stdint.h>

// configuration of PIN

#define HX711_DOUT_PIN  GPIO_PIN_0   // DOUT -> PD0
#define HX711_SCK_PIN   GPIO_PIN_1   // PD_SCK -> PD1
#define HX711_GPIO_PORT GPIOD

// structure type emmumeration for different gain channel

typedef enum{
	HX711_CHANNEL_A_GAIN_128 = 0,   // canal A, gain 128
	HX711_CHANNEL_B_GAIN_32  = 1,   // canal B, gain 32
	HX711_CHANNEL_A_GAIN_64  = 2,   // canal A, gain 64

}hx711_channel;

// initialisation function
void HX711_Init(void);

// function to read the values 24 bits signed of HX711

int32_t HX711_ReadRaw(hx711_channel channel);


//function to check if the date are reading to read
 uint8_t HX711_IsReady (void);



#endif /* HX711_H_ */
