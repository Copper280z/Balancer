#include <Arduino.h>
// #include <SimpleFOC.h>
#include "BLDCMotor.h"

#include "main.h"



// #define MPU6050_ADDR 0xD1
#define MPU6050_I2C hi2c1
#include "mpu6050.h"

volatile unsigned long imu_counter=0;


MPU6050_t imu;
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    imu.Accel_X_RAW = (int16_t)(imu.DMA_buf[0] << 8 | imu.DMA_buf[1]);
    imu.Accel_Y_RAW = (int16_t)(imu.DMA_buf[2] << 8 | imu.DMA_buf[3]);
    imu.Accel_Z_RAW = (int16_t)(imu.DMA_buf[4] << 8 | imu.DMA_buf[5]);
    imu.RAW_temp    = (int16_t)(imu.DMA_buf[6] << 8 | imu.DMA_buf[7]);
    imu.Gyro_X_RAW  = (int16_t)(imu.DMA_buf[8] << 8 | imu.DMA_buf[9]);
    imu.Gyro_Y_RAW  = (int16_t)(imu.DMA_buf[10] << 8 | imu.DMA_buf[11]);
    imu.Gyro_Z_RAW  = (int16_t)(imu.DMA_buf[12] << 8 | imu.DMA_buf[13]);
    MPU6050_StartDMAXfer(hi2c, &imu);
    imu_counter+=1;
}


char uart_buffer[128];
uint32_t loop_counter=0;

// BLDCDriver3PWM driver_1 = BLDCDriver3PWM(PB1, PB0, PA7);
// BLDCDriver3PWM driver_2 = BLDCDriver3PWM(PA6, PA3, PA2);

// BLDCMotor motor_1 = BLDCMotor(11, 9.75);
// BLDCMotor motor_2 = BLDCMotor(11, 9.75);

// Commander command = Commander(Serial);
// void onMotorA(char* cmd){ command.motor(&motor_1, cmd); }
// void onMotorB(char* cmd){ command.motor(&motor_2, cmd); }

void setup() {

  // HAL_Init();
  // SystemClock_Config();
  MX_DMA_Init();
  MX_I2C1_Init();
  // MX_I2C2_Init();
  Serial.begin(5000000);
  // while (!Serial) {};



  // Serial.printf("IMU init returned: %d\n", ret);
  uint8_t check=0;
  uint8_t Data;

  // check device ID WHO_AM_I

  HAL_StatusTypeDef i2cret = HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x75, 1, &check, 1, 100);

  Serial.printf("IMU hi2c1 0xD0 who_am_I returned: %d - %d\n", i2cret, check); 
  delay(1000);
  check=0;
  i2cret = HAL_I2C_Mem_Read(&hi2c1, 0xD2, 0x75, 1, &check, 1, 100);
  Serial.printf("IMU hi2c1 0xD1 who_am_I returned: %d - %d\n", i2cret, check);  
  delay(1000);
  // check=0;
  // i2cret = HAL_I2C_Mem_Read(&hi2c2, 0xD0, 0x75, 1, &check, 1, 100);
  // Serial.printf("IMU hi2c2 0xD0 who_am_I returned: %d - %d\n", i2cret, check); 
  // delay(1000);
  // check=0;
  // i2cret = HAL_I2C_Mem_Read(&hi2c2, 0xD2, 0x75, 1, &check, 1, 100);
  // Serial.printf("IMU hi2c2 0xD1 who_am_I returned: %d - %d\n", i2cret, check);
  // delay(5000);

  uint8_t ret = MPU6050_Init(&hi2c1, (MPU6050_t*)&imu);
  if (ret != 0) {
    delay(5000);
    Serial.println("IMU Initialization failed");
    delay(5000);
  }
  // driver_1.pwm_frequency = 20000;
  // driver_1.voltage_power_supply = 12;
  // driver_1.voltage_limit = driver_1.voltage_power_supply/2;
  // driver_1.init();

  // driver_2.pwm_frequency = 20000;
  // driver_2.voltage_power_supply = 12;
  // driver_2.voltage_limit = driver_1.voltage_power_supply/2;
  // driver_2.init();

  // motor_1.useMonitoring(Serial);
  // motor_2.useMonitoring(Serial);

  // command.add('A', onMotorA, "motor_A");
  // command.add('B', onMotorB, "motor_B");

  // motor_1.linkDriver(&driver_1);
  // motor_1.controller = MotionControlType::velocity_openloop;
  // motor_1.init();
  // // motor_1.initFOC();

  // motor_2.linkDriver(&driver_2);
  // motor_2.controller = MotionControlType::velocity_openloop;
  // motor_2.init();
  // // motor_2.initFOC();

}

void loop() {
  // MPU6050_StartDMAXfer(&hi2c1, &imu);
  // HAL_I2C_MemRxCpltCallback(&hi2c1);
  MPU6050_Read_Accel(&hi2c1, (MPU6050_t*)&imu);

  int str_bytes = snprintf(uart_buffer, 128, "%lu-  X: %.3f - Y: %.3f - Z: %.3f\r\n",(unsigned long) imu_counter, imu.Ax, imu.Ay, imu.Az);
  Serial.write(uart_buffer,str_bytes);
  // if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
  // {
  //   MPU6050_StartDMAXfer(&hi2c1, &imu);
  // } 
  // motor_1.move();
  // motor_2.move();

  // motor_1.loopFOC();
  // motor_2.loopFOC();

  // command.run();

  loop_counter+=1;

}