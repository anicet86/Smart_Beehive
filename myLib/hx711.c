/*
 * hx711.c
 *
 *  Created on: Mar 14, 2026
 *      Author: kemazhu
 *      implemaentation driver of HX711
 */


#include "hx711.h"
#include <stdint.h>
#include "stm32f407xx.h"



// function for the delay in miliseconde

static void delay_ms(uint32_t us){

	volatile uint32_t count = us * 42 ;
	while(count --) {__NOP();}
}

// function of initialisation

void HX711_Init(void){

	RCC ->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock
	// configuration PD0->DOUT in input mode
	GPIOD->MODER &= ~(3U << (2*0)) ;    //  set PD0 as input
	GPIOD->PUPDR &= ~(3U << (2*0));    // set input No pull-up pull-down

	// configuration PD1->PD_SCK in output
	GPIOD->MODER &= ~(3U << (2*1));  //  reset PD1
	GPIOD->OTYPER &= ~(1U << 1);     // set as output push-pull
	GPIOD->OSPEEDR |= (2U << (2*1));   //  SET PD0 high speed

	// put PD_SCK in low state
	GPIOD->BSRR =  (1U << 17);
}
// that funtion verify if date HX711 is ready
uint8_t HX711_IsReady(void){
	return (GPIOD->IDR & (1U << 0) ? 1 : 0);
}

// that function is use to choice channel
int32_t HX711_ReadRaw(hx711_channel channel){

	uint32_t data = 0;
	uint8_t i ;

  while (HX711_IsReady()) {

    	};   //  wait that DOUT go to low state


    //read 24 bits
    for (i = 0; i < 24; i++){
    	GPIOD->BSRR = (1U << 1);  // PD_SCK HIGH
    	delay_ms(1);

    	data  <<=1;
    	// read DOUT
    	if (GPIOD->IDR & (1U << 0)){

    		data |=1 ;
    	}
    	GPIOD->BSRR = (1U << 17);  // PD_SCK LOW
    	delay_ms(1);
    }

    //  apply the channel /gain

   uint8_t pulses = 1;  //  default CHANNEL_A_GAIN_128 ;
   switch (channel) {
	   case HX711_CHANNEL_A_GAIN_128 : pulses = 1; break ;
	   case HX711_CHANNEL_B_GAIN_32  : pulses = 2; break ;
	   case HX711_CHANNEL_A_GAIN_64  : pulses = 3; break ;
   }

   for (i = 0; i < pulses; i++){

	  GPIOD ->BSRR = (1U << 1);
	  delay_ms(1);
	  GPIOD->BSRR  = (1U << 17);
	  delay_ms(1);
   }
   //  conversion in signed integer
   if (data & 0x800000){
	   data |=0xFF000000 ;// extension of signe
   }

   return (int32_t)data ;


}


