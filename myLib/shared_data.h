/*
 * shared_data
 *
 *  Created on: Nov 29, 2025
 *      Author: kemazhu
 */

#ifndef SHARED_DATA_H_
#define SHARED_DATA_H_

#include "FreeRTOS.h"
#include "semphr.h"
//#include "nmea.h"
//#include "gps.h"
// Variables partagées

//----data DHT22-----------//
extern float g_temperature  ; //dht.dht_temperature;
extern float g_humidity ;//dht.dht_humidity;
//------------------------------------//

//----data DS10B20-----------//
extern float g_temperature2; // temperature of DS18B20
//--------------------------------------//

//----data MPU6050-----------
extern volatile float g_accel_Xout;
extern volatile float g_accel_Yout ;
extern volatile float g_accel_Zout;
extern volatile float g_gyr_Xout ;
extern volatile float g_gyr_Yout ;
extern volatile float g_gyr_Zout ;
extern volatile float g_magnitude ;
extern uint8_t g_shock_detected ;
extern float g_pitch ;
extern float g_roll ;


//----data ADXL335-----------
extern float g_acc_x;
extern float g_acc_y;
extern float g_acc_z;
extern float g_inclinaisonX ;
extern float g_inclinaisonY;

//----------data INMP411-----------------//
extern volatile float g_bee_tone_energy;

//------------data HCSR04---------------//
extern uint16_t g_distance_cm ;
extern uint16_t g_distance_m ;

//---------------data GPS--------------------//
extern float g_latitude;
extern float g_longitude;
extern char g_lat_hemisphere;
extern char g_lon_hemisphere;
extern uint8_t g_gps_valid;

//--------------HX711------------------------//
extern float g_weight_kg ;
extern int32_t g_weight_raw;


//---------------semaphore-----------------//
extern SemaphoreHandle_t xSensorMutex;

#endif /* SHARED_DATA_H_ */
