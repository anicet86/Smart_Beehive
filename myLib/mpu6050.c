/*
 * mpu6050.c
 *
 *  Created on: Jan 10, 2026
 *      Author: kemazhu
 */

//Header Files
#include "MPU6050.h"
#include "i2c1.h"
#include "delay1.h"


// Facteurs de sensibilité (par défaut: ±2g, ±250°/s)
static float accel_lsb_per_g = 16384.0f;
static float gyro_lsb_per_dps = 131.0f;

// Offsets de calibration
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

// Timeout (en itérations) – ajuste selon la fréquence CPU
#define I2C_TIMEOUT 10000

// --------------------------------------------------
// Lecture sécurisée multi-octets avec timeout interne
// --------------------------------------------------
static int8_t mpu6050_i2c_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    // On va utiliser ta fonction i2c_ReadMulti, mais on suppose qu'elle est SANS TIMEOUT
    // Donc on ajoute une sécurité ici : vérifier que le bus n'est pas bloqué

    volatile uint32_t timeout = I2C_TIMEOUT;
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if (--timeout == 0) return -1;
    }

    i2c_ReadMulti(MPU6050_I2C_ADDR, reg, len, data);
    return 0;
}

// --------------------------------------------------
// Écriture sécurisée
// --------------------------------------------------
static int8_t mpu6050_i2c_write_reg(uint8_t reg, uint8_t data)
{
    volatile uint32_t timeout = I2C_TIMEOUT;
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if (--timeout == 0) return -1;
    }

    i2c_writeByte(MPU6050_I2C_ADDR, reg, data);
    return 0;
}

// --------------------------------------------------
// Initialisation
// --------------------------------------------------
int8_t MPU6050_Init(void)
{
    // Réveiller le capteur
    if (mpu6050_i2c_write_reg(MPU6050_PWR_MGMT_1, 0x00) != 0) {
        return -1;
    }

    // Optionnel : configurer filtre, plage, etc.
    // Pour l'instant, on garde les valeurs par défaut

    return 0;
}

// --------------------------------------------------
// Test de connexion
// --------------------------------------------------
int8_t MPU6050_TestConnection(void)
{
    uint8_t whoami;
    if (i2c_readByte(MPU6050_I2C_ADDR, MPU6050_WHO_AM_I, &whoami) != 0) {
        return -1;
    }
    return (whoami == 0x68) ? 0 : -1;
}

// --------------------------------------------------
// Lecture des données (modèle DHT22)
// --------------------------------------------------
int8_t MPU6050_Read(MPU6050_Data_t *data)
{
    if (data == NULL) return -1;

    uint8_t buffer[14];
    if (mpu6050_i2c_read_reg(MPU6050_ACCEL_XOUT_H, buffer, 14) != 0) {
        return -1;
    }

    // Extraire les données brutes (big-endian)
    int16_t ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t az = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Appliquer scaling + calibration
    data->accel_x = (float)ax / accel_lsb_per_g;
    data->accel_y = (float)ay / accel_lsb_per_g;
    data->accel_z = (float)az / accel_lsb_per_g;

    data->gyro_x = ((float)gx - gyro_offset_x) / gyro_lsb_per_dps;
    data->gyro_y = ((float)gy - gyro_offset_y) / gyro_lsb_per_dps;
    data->gyro_z = ((float)gz - gyro_offset_z) / gyro_lsb_per_dps;

    return 0; // succès
}

// --------------------------------------------------
// Calibration du gyroscope (à appeler à l'arrêt)
// --------------------------------------------------
void MPU6050_CalibrateGyro(int num_samples)
{
    if (num_samples <= 0) num_samples = 100;

    float sum_x = 0, sum_y = 0, sum_z = 0;
    MPU6050_Data_t sample;

    for (int i = 0; i < num_samples; i++) {
        if (MPU6050_Read(&sample) == 0) {
            sum_x += sample.gyro_x;
            sum_y += sample.gyro_y;
            sum_z += sample.gyro_z;
        }
        // Petit délai pour laisser le temps au capteur
        for (volatile int j = 0; j < 1000; j++);
    }

    gyro_offset_x = sum_x / num_samples;
    gyro_offset_y = sum_y / num_samples;
    gyro_offset_z = sum_z / num_samples;
}
