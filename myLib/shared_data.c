/*
 * GlobalData.c
 *
 *  Created on: Nov 29, 2025
 *      Author: kemazhu
 */
#include "shared_data.h"

// Définition des variables globales

//------------data dht22-------------//
 float g_temperature = 0.0f;
 float g_humidity = 0.0f;


//-----------data ds18b20-------------//
float g_temperature2 = 0.0f;
//--------------------------------------//

//---------data mpu6050---------//
volatile float g_accel_Xout = 0.0f;
volatile float g_accel_Yout = 0.0f;
volatile float g_accel_Zout = 0.0f;
volatile float g_gyr_Xout = 0.0f;
volatile float g_gyr_Yout = 0.0f;
volatile float g_gyr_Zout = 0.0f;
volatile float g_magnitude = 0.0f;
uint8_t g_shock_detected = 0.0f;
float g_pitch = 0.0f;
float g_roll= 0.0f;


//----------------data ADXL335-----------//
float g_acc_x = 0.0f;
float g_acc_y = 0.0f;
float g_acc_z = 0.0f;
float g_inclinaisonX = 0.0f;
float g_inclinaisonY = 0.0f;

//-----------------data INMP441-------------//
volatile float g_bee_tone_energy = 0.0f;

//-------------data HCSR04------------------//
uint16_t g_distance_cm = 0.0f;
uint16_t g_distance_m = 0.0f;

//-------data GPS--------------------------------------//

float g_latitude = 0.0f;
float g_longitude = 0.0f;
char g_lat_hemisphere = 0;
char g_lon_hemisphere = 0;
uint8_t g_gps_valid = 0;

//-------------data HX711------------------------//
float g_weight_kg = 0.0f;
int32_t g_weight_raw = 0.0f;


SemaphoreHandle_t xSensorMutex = NULL;
