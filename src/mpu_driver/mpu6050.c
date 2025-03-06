
/**
 * @file mpu6050.c
 * @brief Driver implementation for the MPU-6050 sensor
 */

#include <stdint.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mpu6050);

/**
 * @brief Registers addresses
 */
#define ACCE_X (0x3B)
#define ACCE_Y (0x3D)
#define ACCE_Z (0x3F)
#define TEMP_T (0x41)
#define GYRO_X (0x43)
#define GYRO_Y (0x45)
#define GYRO_Z (0x47)

static const struct i2c_dt_spec i2c0 = I2C_DT_SPEC_GET(DT_NODELABEL(mpu6050));

typedef enum {
  I2C_OK = 0, // Success
  I2C_ERR,    // Bus error
} I2C_Status;

typedef enum {
  I2C_READ,  // Read from I2C device
  I2C_WRITE, // Write to I2C device
} I2C_operation;

bool mpu6050_init(void) {
  // Validate I2C bus is ready
  if (!i2c_is_ready_dt(&i2c0)) {
    LOG_ERR("I2C bus used by the MPU-6050 is not read");
    return false;
  }

  // Wake up the sensor
  i2c_reg_write_byte_dt(&i2c0, 0x6B, 0x00);

  LOG_DBG("I2C bus for the MPU-6050 is ready");
  return true;
}

void mpu6050_read_acce(int16_t *x, int16_t *y, int16_t *z) {
  // Read accelerometer data
  uint8_t reg_addr = ACCE_X;
  int16_t data[3];
  if (i2c_write_read_dt(&i2c0, &reg_addr, sizeof(reg_addr), (void *)data,
                        sizeof(data)) != I2C_OK) {
    LOG_ERR("Unable to read accelerometer data over I2C bus");
  }

  LOG_DBG("MPU-6050 accelerometer data sccessfully read");
  *x = data[0];
  *y = data[1];
  *z = data[2];
}

void mpu6050_read_gyro(int16_t *x, int16_t *y, int16_t *z) {
  // Read gyroscope data
  uint8_t reg_addr = GYRO_X;
  int16_t data[3];
  if (i2c_write_read_dt(&i2c0, &reg_addr, sizeof(reg_addr), (void *)data,
                        sizeof(data)) != I2C_OK) {
    LOG_ERR("Unable to read gyroscope data over I2C bus");
  }

  LOG_DBG("MPU-6050 gyroscope data sccessfully read");
  *x = data[0];
  *y = data[1];
  *z = data[2];
}

void mpu6050_read_temp(int16_t *t) {
  // Read temperature data
  uint8_t reg_addr = TEMP_T;
  if (i2c_write_read_dt(&i2c0, &reg_addr, sizeof(reg_addr), (void *)t,
                        sizeof(t)) != I2C_OK) {
    LOG_ERR("Unable to read temperature data over I2C bus");
  }
  LOG_DBG("MPU-6050 temperature data sccessfully read");
}
