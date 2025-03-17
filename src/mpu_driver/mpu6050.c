
/**
 * @file mpu6050.c
 * @brief Driver implementation for the MPU-6050 sensor
 */

#include "mpu6050.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(mpu6050);

const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);

static int prvProcessMpu6050(const struct device *dev,
                             struct mpu6050_data *data) {
  struct sensor_value temperature;
  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  int rc = sensor_sample_fetch(dev);

  if (rc == 0) {
    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  }
  if (rc == 0) {
    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
  }
  if (rc == 0) {
    rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
  }
  if (rc == 0) {
    data->temp = sensor_value_to_double(&temperature);
    data->acce[X_INDEX] = sensor_value_to_double(&accel[0]);
    data->acce[Y_INDEX] = sensor_value_to_double(&accel[1]);
    data->acce[Z_INDEX] = sensor_value_to_double(&accel[2]);
    data->gyro[X_INDEX] = sensor_value_to_double(&gyro[0]);
    data->gyro[Y_INDEX] = sensor_value_to_double(&gyro[1]);
    data->gyro[Z_INDEX] = sensor_value_to_double(&gyro[2]);
  } else {
    LOG_ERR("sample fetch/get failed: %d", rc);
  }

  return rc;
}

#ifdef CONFIG_MPU6050_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu6050_drdy(const struct device *dev,
                                const struct sensor_trigger *trig) {
  int rc = prvProcessMpu6050(dev);

  if (rc != 0) {
    LOG_ERR("cancelling trigger due to failure: %d", rc);
    (void)sensor_trigger_set(dev, trig, NULL);
    return;
  }
}
#endif

bool mpu6050_sensor_init(void) {
  if (!device_is_ready(mpu6050)) {
    LOG_ERR("Device %s is not ready", mpu6050->name);
    return false;
  }

#ifdef CONFIG_MPU6050_TRIGGER
  trigger = (struct sensor_trigger){
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_ALL,
  };
  if (sensor_trigger_set(mpu6050, &trigger, handle_mpu6050_drdy) < 0) {
    LOG_ERR("Cannot configure trigger");
    return false;
  }
  LOG_INF("Configured for triggered sampling");
#endif

  LOG_INF("Device %s ready", mpu6050->name);
  return true;
}

void mpu6050_get_data(struct mpu6050_data *const data) {
  if (!IS_ENABLED(CONFIG_MPU6050_TRIGGER)) {
    prvProcessMpu6050(mpu6050, data);
  }
}
