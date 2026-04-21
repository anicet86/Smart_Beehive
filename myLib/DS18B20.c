/*
 * DS18B20.c
 *
 *  Created on: Nov 22, 2025
 *      Author: kemazhu
 */

// ds18b20.c
#include "stm32f407xx.h"
#include "ds18b20.h"
#include "Delay.h"
#include "DS18B20.h"
//#include "delay1.h"
// Broche : PA8
#define DS18B20_PIN        GPIO_PIN_8
#define DS18B20_PORT       GPIOA

// Commandes 1-Wire
#define CMD_RESET          0xF0
#define CMD_SKIP_ROM       0xCC
#define CMD_CONVERT_T      0x44
#define CMD_READ_SCRATCHPAD 0xBE

// ------------------------------------------------------------------
// Fonctions bas niveau 1-Wire
// ------------------------------------------------------------------

static void DS18B20_Pin_Output(void)
{
    GPIOA->MODER &= ~(GPIO_MODER_MODER8);       // Clear mode bits
    GPIOA->MODER |= GPIO_MODER_MODER8_0;        // Output mode (01)
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;         // Push-pull (mais on simule open-drain via logiciel)
}

static void DS18B20_Pin_Input(void)
{
    GPIOA->MODER &= ~(GPIO_MODER_MODER8);       // Input mode (00)
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0;        // Pull-up
}

static void DS18B20_WriteBit(uint8_t bit)
{
    if (bit) {
        // Écrire 1 : >1µs bas, puis relâcher
        DS18B20_Pin_Output();
        GPIOA->BSRR = DS18B20_PIN << 16; // Set output low
       // delay_us1(1);
        TimerDelay_us(1) ;//delay_us(1);
        DS18B20_Pin_Input(); // Relâcher → pull-up remonte à 1
        //delay_us1(2);
        TimerDelay_us(60);//delay_us(60);
    } else {
        // Écrire 0 : 60µs bas
        DS18B20_Pin_Output();
        GPIOA->BSRR = DS18B20_PIN << 16; // Low
        //delay_us1(60);
       TimerDelay_us(60); //  delay_us(60)
        DS18B20_Pin_Input(); // Relâcher
        //delay_us1(1);
        TimerDelay_us(1); //  delay_us(1);
    }
}

static uint8_t DS18B20_ReadBit(void)
{
    uint8_t bit;
    DS18B20_Pin_Output();
    GPIOA->BSRR = DS18B20_PIN << 16; // Pull low
    //delay_us1(1);
     TimerDelay_us(1); //delay_us(1);
    DS18B20_Pin_Input(); // Relâcher
    //delay_us1(10);
    TimerDelay_us(10);// delay_us(10); // Temps de récupération
    bit = (GPIOA->IDR & DS18B20_PIN) ? 1 : 0;
    //delay_us1(50);
    TimerDelay_us(50);//  delay_us(50);
    return bit;
}

static void DS18B20_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        DS18B20_WriteBit(byte & 1);
        byte >>= 1;
    }
}

static uint8_t DS18B20_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte |= (DS18B20_ReadBit() << i);
    }
    return byte;
}

static uint8_t DS18B20_Reset(void)
{
    uint8_t presence = 0;

    DS18B20_Pin_Output();
    GPIOA->BSRR = DS18B20_PIN << 16; // LOW
    //delay_us1(480);
    TimerDelay_us(480);// delay_us(480);
    DS18B20_Pin_Input(); // Relâcher
   // delay_us1(60);
   TimerDelay_us(70);  //delay_us(70);
    presence = (GPIOA->IDR & DS18B20_PIN) ? 1 : 0; // 0 = présence détectée
    //delay_us1(70);
    TimerDelay_us(70); //  delay_us(410);
    return presence; // 0 = OK, 1 = pas de réponse
}

// ------------------------------------------------------------------
// CRC8 pour vérification des données
// ------------------------------------------------------------------
static uint8_t DS18B20_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t fb;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 8; j; j--) {
            fb = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (fb) crc ^= 0x8C;
            byte >>= 1;
        }
    }
    return crc;
}

// ------------------------------------------------------------------
// Fonctions publiques
// ------------------------------------------------------------------

void DS18B20_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Activer horloge GPIOA
    RCC->CFGR &= ~RCC_CFGR_MCO1;
    // Configuration initiale : input avec pull-up (état repos)
    GPIOA->MODER &= ~(GPIO_MODER_MODER8);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0; // Pull-up

    // ⚠️ Une résistance de pull-up externe de 4.7kΩ est fortement recommandée entre PA8 et VCC
}

uint8_t DS18B20_ReadTemperature(DS18B20_Data *data)
{
    if (DS18B20_Reset()) {
        return DS18B20_NO_DEVICE;
    }

    // Lancer conversion
    DS18B20_WriteByte(CMD_SKIP_ROM);
    DS18B20_WriteByte(CMD_CONVERT_T);
    TimerDelay_ms(750); // delay_ms(750); // Attendre conversion (max 750 ms pour 12-bit)

    // Relire après conversion
    if (DS18B20_Reset()) {
        return DS18B20_NO_DEVICE;
    }

    DS18B20_WriteByte(CMD_SKIP_ROM);
    DS18B20_WriteByte(CMD_READ_SCRATCHPAD);

    // Lire 9 octets du scratchpad
    uint8_t scratchpad[9];
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = DS18B20_ReadByte();
    }

    // Vérifier CRC
    if (DS18B20_CRC8(scratchpad, 8) != scratchpad[8]) {
        return DS18B20_CRC_ERROR;
    }

    // Extraire température (16-bit little-endian)
    int16_t raw = (scratchpad[1] << 8) | scratchpad[0];
    data->temperature = (float)raw / 16.0f;

    // ROM : on ne l'a pas lue ici (SKIP_ROM utilisé)
    // Pour lire la ROM, il faudrait utiliser CMD_READ_ROM au lieu de SKIP_ROM
    // Ici, on remplit avec 0xFF pour indiquer "non lu"
    for (int i = 0; i < 8; i++) {
        data->rom[i] = 0xFF;
    }

    data->last_status = DS18B20_OK;
    return DS18B20_OK;
}
