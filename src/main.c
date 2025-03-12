/**
 * @file main.c
 * @brief Zephyr Demo Application for the nRF52 DK Board
 */

#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_def.h"
#include "demo_task1.h"
#include "demo_task2.h"
#include "mpu6050.h"

LOG_MODULE_REGISTER(main);

bool prvInitBoard(void) {
  uint8_t ret;

  if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1) ||
      !gpio_is_ready_dt(&led2) || !gpio_is_ready_dt(&led3) ||
      !gpio_is_ready_dt(&btn0) || !gpio_is_ready_dt(&btn1) ||
      !gpio_is_ready_dt(&btn2) || !gpio_is_ready_dt(&btn3)) {
    return false;
  }
  ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return false;
  }
  ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return false;
  }
  ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return false;
  }
  ret = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return false;
  }
  ret = gpio_pin_configure_dt(&btn0, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return false;
  }
  ret = gpio_pin_configure_dt(&btn1, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return false;
  }
  ret = gpio_pin_configure_dt(&btn2, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return false;
  }
  ret = gpio_pin_configure_dt(&btn3, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return false;
  }

  return true;
}

int main(void) {
  // Initialize hardware
  if (!prvInitBoard()) {
    LOG_ERR("Failed to initialize board");
    return 0;
  }

  if (!mpu6050_sensor_init()) {
    LOG_ERR("Failed to initialize MPU-6050 sensor");
    return 0;
  }

  // Start tasks
  task1_init();
  task2_init();

  LOG_INF("--- Application is starting ---");

  while (1) {
    k_sleep(K_SECONDS(2));
    mpu6050_get_data();
  }

  return 0;
}
