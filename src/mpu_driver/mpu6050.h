
/**
 * @file mpu6050.h
 * @brief Driver API for the MPU-6050 sensor
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

bool mpu6050_init(void);

void mpu6050_read_acce(int16_t *x, int16_t *y, int16_t *z);

void mpu6050_read_gyro(int16_t *x, int16_t *y, int16_t *z);

void mpu6050_read_temp(int16_t *t);

#endif // _MPU6050_H_
