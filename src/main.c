/**
 * @file main.c
 * @brief Zephyr Demo Application for the nRF52 DK Board
 */

#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/llext/buf_loader.h>
#include <zephyr/llext/llext.h>
#include <zephyr/logging/log.h>

#include "ble_task.h"
#include "board_def.h"
#include "demo_task1.h"
#include "demo_task2.h"
#include "sensor_task.h"
#include "uart_task.h"

LOG_MODULE_REGISTER(main);

static uint8_t llext_buf[] = {
#include "user_app_ext.inc"
};

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

  // Start tasks
  if (!task1_init()) {
    LOG_ERR("Failed to init task 1");
  }
  if (!task2_init()) {
    LOG_ERR("Failed to init task 2");
  }
  if (!task_sensor_init()) {
    LOG_ERR("Failed to init sensor task");
  }
#ifdef CONFIG_BLE_SENSOR_DATA
  if (!task_ble_init()) {
    LOG_ERR("Failed to init BLE task");
  }
#endif
#ifdef CONFIG_UART_SENSOR_DATA
  if (!task_uart_init()) {
    LOG_ERR("Failed to init UART task");
  }
#endif

  LOG_INF("--- OS is starting ---");
  k_msleep(1000);

  size_t llext_buf_len = ARRAY_SIZE(llext_buf);
  struct llext_buf_loader buf_loader =
      LLEXT_PERSISTENT_BUF_LOADER(llext_buf, llext_buf_len);
  struct llext_loader *ldr = &buf_loader.loader;

  struct llext_load_param ldr_parm = LLEXT_LOAD_PARAM_DEFAULT;
  struct llext *ext;

  if (0 != llext_load(ldr, "User app", &ext, &ldr_parm)) {
    LOG_ERR("Failed to load extension");
    return -1;
  }

  void (*user_app_fn)() = llext_find_sym(&ext->exp_tab, "UserAppEntryPoint");

  if (user_app_fn == NULL) {
    LOG_ERR("Failed to find symbol");
    return -1;
  }

  LOG_INF("--- User application is starting ---");
  user_app_fn();

  while (1) {
    k_sleep(K_FOREVER);
  }

  llext_unload(&ext);
  return 0;
}
