/*
 * mpu5060.c
 *
 *  Created on: Jan 8, 2026
 *      Author: kemazhu
 */

// le MUP650 est un capteur qui mesure l'acceleration et l'inclinaison

// i2c.c
#include <i2c1.h>
#include <stdio.h>
#include "stm32f4xx.h"

#define I2C_TIMEOUT 10000

uint8_t inited = 0;

void i2c_init(void)
{
    if (inited == 0)
    {
        // Activer horloges
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   // I2C1

        // Configurer PB6 (SCL) et PB7 (SDA) en Alternate Function
        GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // AF mode
        GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;      // Open-drain
        GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7; // High speed
        GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0; // Pull-up

        // AF1 pour PB6 et PB7 → bits [27:24] pour PB7, [23:20] pour PB6
        GPIOB->AFR[0] |= (1U << 24) | (1U << 28); // PB6 = AFR[0][27:24], PB7 = AFR[0][31:28]

        // Reset I2C1
        I2C1->CR1 |= I2C_CR1_SWRST;
        I2C1->CR1 &= ~I2C_CR1_SWRST;

        // Configuration pour 100 kHz (PCLK1 = 16 MHz)
        I2C1->CR2 = 16;           // PCLK1 = 16 MHz
        I2C1->CCR = 80;           // 16e6 / (2 * 100e3) = 80
        I2C1->TRISE = 17;         // 16 + 1 = 17

        I2C1->CR1 |= I2C_CR1_PE;  // Activer I2C
        inited = 1;
    }
}

// Lecture d'un octet
char i2c_readByte(uint8_t saddr, uint8_t maddr, uint8_t *data)
{
    // Écrire l'adresse du registre
    i2c_writeByte(saddr, maddr, 0); // on ne se soucie pas de la donnée

    // Générer un START répété
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // Envoyer l'adresse + bit lecture
    I2C1->DR = (saddr << 1) | 0x01;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2; // Effacer ADDR

    // NACK + STOP après 1 octet
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;

    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    *data = I2C1->DR;
    return 0;
}

// Écriture d'un octet
void i2c_writeByte(uint8_t saddr, uint8_t maddr, uint8_t data)
{
	volatile uint32_t timeout;

	    // Attendre que le bus soit libre
	    timeout = I2C_TIMEOUT;
	    while ((I2C1->SR2 & I2C_SR2_BUSY) && --timeout);
	    if (timeout == 0) return; // Échec silencieux ou gérer erreur

	    // START
	    I2C1->CR1 |= I2C_CR1_START;
	    timeout = I2C_TIMEOUT;
	    while (!(I2C1->SR1 & I2C_SR1_SB) && --timeout);
	    if (timeout == 0) return;
	    // Adresse + écriture
	     I2C1->DR = saddr << 1;
	     timeout = I2C_TIMEOUT;
	     while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout);
	     if (timeout == 0) return;
	     (void)I2C1->SR2;
	     // Registre
	        I2C1->DR = maddr;
	        timeout = I2C_TIMEOUT;
	        while (!(I2C1->SR1 & I2C_SR1_TXE) && --timeout);
	        if (timeout == 0) return;

	        // Donnée
	        I2C1->DR = data;
	        timeout = I2C_TIMEOUT;
	        while (!(I2C1->SR1 & I2C_SR1_BTF) && --timeout);
	        if (timeout == 0) return;

	        I2C1->CR1 |= I2C_CR1_STOP;
}

// Écriture multiple
void i2c_WriteMulti(uint8_t saddr, uint8_t maddr, uint8_t *buffer, int length)
{
    while (I2C1->SR2 & I2C_SR2_BUSY);

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = saddr << 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    I2C1->DR = maddr;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    for (int i = 0; i < length; i++)
    {
        I2C1->DR = buffer[i];
        while (!(I2C1->SR1 & I2C_SR1_BTF));
    }

    I2C1->CR1 |= I2C_CR1_STOP;
}

// Lecture multiple
void i2c_ReadMulti(uint8_t saddr, uint8_t maddr, int n, uint8_t* data)
{
	 if (n <= 0) return;
	    volatile uint32_t timeout;

	    // Écrire le registre à lire
	    i2c_writeByte(saddr, maddr, 0);

	    // START répété
	    timeout = I2C_TIMEOUT;
	    while ((I2C1->SR2 & I2C_SR2_BUSY) && --timeout);
	    if (timeout == 0) return;
	    I2C1->DR = (saddr << 1) | 0x01;
	    timeout = I2C_TIMEOUT;
	     while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout);
	     if (timeout == 0) return;
	     (void)I2C1->SR2;
	     if (n == 1) {
	          I2C1->CR1 &= ~I2C_CR1_ACK;
	          I2C1->CR1 |= I2C_CR1_STOP;
	           timeout = I2C_TIMEOUT;
	            while (!(I2C1->SR1 & I2C_SR1_RXNE) && --timeout);
	            if (timeout) data[0] = I2C1->DR;
	            I2C1->CR1 |= I2C_CR1_ACK;
	            return;
	     }
	     for (int i = 0; i < n - 1; i++) {
	             timeout = I2C_TIMEOUT;
	             while (!(I2C1->SR1 & I2C_SR1_RXNE) && --timeout);
	             if (timeout == 0) return;
	             data[i] = I2C1->DR;
	         }
	     // Dernier octet
	         I2C1->CR1 &= ~I2C_CR1_ACK;
	         I2C1->CR1 |= I2C_CR1_STOP;
	         timeout = I2C_TIMEOUT;
	         while (!(I2C1->SR1 & I2C_SR1_RXNE) && --timeout);
	         if (timeout) data[n-1] = I2C1->DR;

	         I2C1->CR1 |= I2C_CR1_ACK;


}

// Scan du bus I2C
void i2c_bus_scan(void)
{
    printf("Scanning I2C bus...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        while (I2C1->SR2 & I2C_SR2_BUSY);

        I2C1->CR1 |= I2C_CR1_START;
        while (!(I2C1->SR1 & I2C_SR1_SB));

        I2C1->DR = addr << 1;
        volatile uint32_t timeout = 10000;
        while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && --timeout);

        if (I2C1->SR1 & I2C_SR1_ADDR)
        {
            (void)I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;
            printf("Device found at 0x%02X\r\n", addr);
        }
        else if (I2C1->SR1 & I2C_SR1_AF)
        {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
    }
}
