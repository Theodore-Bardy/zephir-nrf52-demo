
/**
 * @file mpu6050.c
 * @brief Driver implementation for the MPU-6050 sensor
 */

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(mpu6050);

const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);

static double ax, ay, az, gx, gy, gz, t;

static int prvProcessMpu6050(const struct device *dev) {
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
    t = sensor_value_to_double(&temperature);
    ax = sensor_value_to_double(&accel[0]);
    ay = sensor_value_to_double(&accel[1]);
    az = sensor_value_to_double(&accel[2]);
    gx = sensor_value_to_double(&gyro[0]);
    gy = sensor_value_to_double(&gyro[1]);
    gz = sensor_value_to_double(&gyro[2]);
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

void mpu6050_get_data(void) {
  if (!IS_ENABLED(CONFIG_MPU6050_TRIGGER)) {
    prvProcessMpu6050(mpu6050);
  }

  LOG_INF("\n\r  temp  %lf Cel\n\r"
          "  accel %lf %lf %lf m/s/s\n\r"
          "  gyro  %lf %lf %lf rad/s",
          t, ax, ay, az, gx, gy, gz);
}
