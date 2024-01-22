/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *  Modified for DMA 1/14/24
*/

/*
 * In order to use this with DMA, you need to implement some method of safely moving the
 * DMA buffer data into the *_RAW members of the struct. I chose to use the 
 * MemRxCplt callback to do this, along with making the data stuct available globally.
 * However, you're free to do it however you want.
 *
 * MPU6050_t imu;
 * void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
 *     imu.Accel_X_RAW = (int16_t)(imu.DMA_buf[0] << 8 | imu.DMA_buf[1]);
 *     imu.Accel_Y_RAW = (int16_t)(imu.DMA_buf[2] << 8 | imu.DMA_buf[3]);
 *     imu.Accel_Z_RAW = (int16_t)(imu.DMA_buf[4] << 8 | imu.DMA_buf[5]);
 *     imu.RAW_temp    = (int16_t)(imu.DMA_buf[6] << 8 | imu.DMA_buf[7]);
 *     imu.Gyro_X_RAW  = (int16_t)(imu.DMA_buf[8] << 8 | imu.DMA_buf[9]);
 *     imu.Gyro_Y_RAW  = (int16_t)(imu.DMA_buf[10] << 8 | imu.DMA_buf[11]);
 *     imu.Gyro_Z_RAW  = (int16_t)(imu.DMA_buf[12] << 8 | imu.DMA_buf[13]);
 *     MPU6050_StartDMAXfer(hi2c, &imu);
 * }
*/
#ifndef INC_GY521_H_
#define INC_GY521_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <math.h>
#include <stdint.h>
#include "i2c.h"

// MPU6050 structure
typedef struct
{
    uint8_t DMA_buf[14];

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;
    
    int16_t RAW_temp;
    float Temperature;

    float KalmanAngleX;
    float KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t* DataStruct);

void MPU6050_StartDMAXfer(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);


#ifdef __cplusplus
}
#endif

#endif /* INC_GY521_H_ */
