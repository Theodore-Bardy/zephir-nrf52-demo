
/**
 * @file mpu6050.h
 * @brief Driver API for the MPU-6050 sensor
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "zephyr/dsp/types.h"
#include <stdbool.h>
#include <stdint.h>

#define X_INDEX (0)
#define Y_INDEX (1)
#define Z_INDEX (2)

struct mpu6050_data {
  float64_t temp;
  float64_t acce[3];
  float64_t gyro[3];
};

bool mpu6050_sensor_init(void);

void mpu6050_get_data(struct mpu6050_data *const data);

#endif // _MPU6050_H_
