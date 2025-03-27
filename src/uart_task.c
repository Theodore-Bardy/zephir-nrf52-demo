#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

#include "mpu6050.h"
#include "uart_task.h"

LOG_MODULE_REGISTER(uart);

ZBUS_CHAN_DECLARE(sensor_data);

// BLE task configuration
#define TASK_UART_STACK_SIZE (256)
#define TASK_UART_PRIORITITY (2)
static void pvrTaskUart(void *sub);
ZBUS_SUBSCRIBER_DEFINE(uart_com_sub, 1);
ZBUS_CHAN_ADD_OBS(sensor_data, uart_com_sub, 1);
K_THREAD_DEFINE(task_uart_id, TASK_UART_STACK_SIZE, pvrTaskUart, &uart_com_sub,
                NULL, NULL, TASK_UART_PRIORITITY, K_ESSENTIAL, 0);

// Synchronization variables
K_SEM_DEFINE(sem_task_uart_start, 0, 1);

bool task_uart_init(void) {
  k_sem_give(&sem_task_uart_start);
  return true;
}

static void pvrTaskUart(void *sub) {
  const struct zbus_channel *channel;
  const struct zbus_observer *data_sub = sub;
  struct mpu6050_data mpu_srv_data = {0};
  int ret;

  k_sem_take(&sem_task_uart_start, K_FOREVER);

  LOG_INF("UART task start...");

  // Wait for data from the sensor task
  while (!zbus_sub_wait(data_sub, &channel, K_FOREVER)) {
    if (&sensor_data != channel) {
      LOG_ERR("Wrong channel triggered: %s (expected: %s)", channel->name,
              sensor_data.name);
      continue;
    }
    ret = zbus_chan_read(channel, &mpu_srv_data, K_NO_WAIT);
    if (ret != 0) {
      LOG_ERR("Unable to read data over channel %s", channel->name);
      continue;
    }

    LOG_INF("\n\r  temp  %lf Cel\n\r"
            "  accel %lf %lf %lf m/s/s\n\r"
            "  gyro  %lf %lf %lf rad/s",
            mpu_srv_data.temp, mpu_srv_data.acce[X_INDEX],
            mpu_srv_data.acce[Y_INDEX], mpu_srv_data.acce[Z_INDEX],
            mpu_srv_data.gyro[X_INDEX], mpu_srv_data.gyro[Y_INDEX],
            mpu_srv_data.gyro[Z_INDEX]);
  }
}
