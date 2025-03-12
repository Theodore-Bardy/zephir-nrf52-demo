
/**
 * @file mpu6050.h
 * @brief Driver API for the MPU-6050 sensor
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdbool.h>

bool mpu6050_sensor_init(void);

void mpu6050_get_data(void);

#endif // _MPU6050_H_
