/*
 * mpu6050.h
 *
 *  Created on: Jan 10, 2026
 *      Author: kemazhu
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2c1.h"
#include <string.h>
#include <stdbool.h>	//Boolean
#include <math.h>			//Pow()

//Define Registers
#define MPU6050_I2C_ADDR        0x68

// Registres
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43

// Structure de données scalées
typedef struct {
    float accel_x;   // en g
    float accel_y;
    float accel_z;
    float gyro_x;    // en °/s
    float gyro_y;
    float gyro_z;
} MPU6050_Data_t;


// Fonctions publiques
int8_t MPU6050_Init(void);
int8_t MPU6050_TestConnection(void);
int8_t MPU6050_Read(MPU6050_Data_t *data); // comme DHT22_Read()
void MPU6050_CalibrateGyro(int num_samples);
#endif /* MPU6050_H_ */
