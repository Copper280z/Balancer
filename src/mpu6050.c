/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
// #ifndef MPU6050_ADDR
#define MPU6050_ADDR 0xD2
// #endif
const uint16_t i2c_timeout = 100;
const float Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

void MPU6050_StartDMAXfer(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1,(uint8_t*)DataStruct->DMA_buf,14);
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t* DataStruct)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        
        MPU6050_StartDMAXfer(I2Cx, DataStruct);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    __disable_irq();
    int16_t temp_Accel_X_RAW = DataStruct->Accel_X_RAW;
    int16_t temp_Accel_Y_RAW = DataStruct->Accel_Y_RAW;
    int16_t temp_Accel_Z_RAW = DataStruct->Accel_Z_RAW;
    __enable_irq();
    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = temp_Accel_X_RAW / 16384.0;
    DataStruct->Ay = temp_Accel_Y_RAW / 16384.0;
    DataStruct->Az = temp_Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    __disable_irq();
    int16_t temp_Gyro_X_RAW = DataStruct->Gyro_X_RAW;
    int16_t temp_Gyro_Y_RAW = DataStruct->Gyro_Y_RAW;
    int16_t temp_Gyro_Z_RAW = DataStruct->Gyro_Z_RAW;
    __enable_irq();

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = temp_Gyro_X_RAW / 131.0;
    DataStruct->Gy = temp_Gyro_Y_RAW / 131.0;
    DataStruct->Gz = temp_Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    __disable_irq();
    int16_t temp_RAW_temp = DataStruct->RAW_temp;
    __enable_irq();
    DataStruct->Temperature = (float)(temp_RAW_temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    __disable_irq();
    int16_t temp_Accel_X_RAW = DataStruct->Accel_X_RAW;
    int16_t temp_Accel_Y_RAW = DataStruct->Accel_Y_RAW;
    int16_t temp_Accel_Z_RAW = DataStruct->Accel_Z_RAW;
    int16_t temp_RAW_temp = DataStruct->RAW_temp;
    int16_t temp_Gyro_X_RAW = DataStruct->Gyro_X_RAW;
    int16_t temp_Gyro_Y_RAW = DataStruct->Gyro_Y_RAW;
    int16_t temp_Gyro_Z_RAW = DataStruct->Gyro_Z_RAW;
    __enable_irq();

    // Kalman angle solve
    float dt = (float)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    float roll;
    float roll_sqrt = sqrtf(
        temp_Accel_X_RAW * temp_Accel_X_RAW + temp_Accel_Z_RAW * temp_Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(temp_Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    float pitch = atan2(-temp_Accel_X_RAW, temp_Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
    float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
