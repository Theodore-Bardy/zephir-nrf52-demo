/**
 * @file main.c
 * @brief Zephyr Demo Application for the nRF52 DK Board
 */

#include <stdint.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble.h"
#include "mpu6050.h"

LOG_MODULE_REGISTER(main);

#define SLEEP_TIME_MS (10000)

/**
 * @brief Hardware used in the demo
 */
static const struct gpio_dt_spec led0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
static const struct gpio_dt_spec led1 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led1), gpios);
static const struct gpio_dt_spec led2 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios);
static const struct gpio_dt_spec led3 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led3), gpios);
static const struct gpio_dt_spec btn0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);
static const struct gpio_dt_spec btn1 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(button1), gpios);
static const struct gpio_dt_spec btn2 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(button2), gpios);
static const struct gpio_dt_spec btn3 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(button3), gpios);

/**
 * @brief Synchronization variables
 */
static struct k_sem btn3_press_sem;

/**
 * @brief Thread macro and variables
 */
#define LED_THREAD_STACK_SIZE (250)
#define LED_THREAD_PRIORITY (3)
K_THREAD_STACK_DEFINE(led_thread_area, LED_THREAD_STACK_SIZE);
static k_tid_t led_thread_id;
static struct k_thread led_thread_cxt;
static void prvLedThread(void *arg0, void *arg1, void *arg2);

#define STAT_THREAD_STACK_SIZE (250)
#define STAT_THREAD_PRIORITY (2)
K_THREAD_STACK_DEFINE(stat_thread_area, STAT_THREAD_STACK_SIZE);
static k_tid_t stat_thread_id;
static struct k_thread stat_thread_cxt;
static void prvStatThread(void *arg0, void *arg1, void *arg2);

#define MPU_THREAD_STACK_SIZE (1024)
#define MPU_THREAD_PRIORITY (4)
K_THREAD_STACK_DEFINE(mpu_thread_area, MPU_THREAD_STACK_SIZE);
static k_tid_t mpu_thread_id;
static struct k_thread mpu_thread_cxt;
static void prvMpuThread(void *arg0, void *arg1, void *arg2);

/**
 * @brief Interrupt structures and callbacks
 */
static struct gpio_callback btn0_cb_data;
static struct gpio_callback btn1_cb_data;
static struct gpio_callback btn2_cb_data;
static struct gpio_callback btn3_cb_data;

void btn0_pressed(const struct device *port, struct gpio_callback *cb,
                  gpio_port_pins_t pins) {
  // Toggle led2 when the btn0 is pressed
  gpio_pin_toggle_dt(&led2);
}

void btn1_pressed(const struct device *port, struct gpio_callback *cb,
                  gpio_port_pins_t pins) {
  // Toggle led3 when the btn1 is pressed
  gpio_pin_toggle_dt(&led3);
}

void btn2_pressed(const struct device *port, struct gpio_callback *cb,
                  gpio_port_pins_t pins) {
  LOG_INF("Monstre!");
}

void btn3_pressed(const struct device *port, struct gpio_callback *cb,
                  gpio_port_pins_t pins) {
  // Give semaphore when the btn3 is pressed
  k_sem_give(&btn3_press_sem);
}

int main(void) {
  // Initialize hardware
  uint8_t ret;

  // if (!mpu6050_init()) {
  //   return 0;
  // }
  if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1) ||
      !gpio_is_ready_dt(&led2) || !gpio_is_ready_dt(&led3) ||
      !gpio_is_ready_dt(&btn0) || !gpio_is_ready_dt(&btn1) ||
      !gpio_is_ready_dt(&btn2) || !gpio_is_ready_dt(&btn3)) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&btn0, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&btn1, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&btn2, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&btn3, GPIO_INPUT | GPIO_PULL_UP);
  if (ret < 0) {
    return 0;
  }

  // Initialize synchronization variables
  k_sem_init(&btn3_press_sem, 0u, 1u);

  // Initialize callbacks
  gpio_init_callback(&btn0_cb_data, btn0_pressed, BIT(btn0.pin));
  gpio_init_callback(&btn1_cb_data, btn1_pressed, BIT(btn1.pin));
  gpio_init_callback(&btn2_cb_data, btn2_pressed, BIT(btn2.pin));
  gpio_init_callback(&btn3_cb_data, btn3_pressed, BIT(btn3.pin));

  gpio_add_callback_dt(&btn0, &btn0_cb_data);
  gpio_add_callback_dt(&btn1, &btn1_cb_data);
  gpio_add_callback_dt(&btn2, &btn2_cb_data);
  gpio_add_callback_dt(&btn3, &btn3_cb_data);

  gpio_pin_interrupt_configure_dt(&btn0, GPIO_INT_EDGE_FALLING);
  gpio_pin_interrupt_configure_dt(&btn1, GPIO_INT_EDGE_FALLING);
  gpio_pin_interrupt_configure_dt(&btn2, GPIO_INT_EDGE_FALLING);
  gpio_pin_interrupt_configure_dt(&btn3, GPIO_INT_EDGE_FALLING);

  // Threads defintion
  led_thread_id = k_thread_create(&led_thread_cxt, led_thread_area,
                                  K_THREAD_STACK_SIZEOF(led_thread_area),
                                  prvLedThread, NULL, NULL, NULL,
                                  LED_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

  // stat_thread_id = k_thread_create(
  //     &stat_thread_cxt, stat_thread_area,
  //     K_THREAD_STACK_SIZEOF(stat_thread_area), prvStatThread, NULL, NULL,
  //     NULL, STAT_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

  // mpu_thread_id = k_thread_create(&mpu_thread_cxt, mpu_thread_area,
  //                                  K_THREAD_STACK_SIZEOF(mpu_thread_area),
  //                                  prvMpuThread, NULL, NULL, NULL,
  //                                  MPU_THREAD_PRIORITY, K_ESSENTIAL,
  //                                  K_NO_WAIT);

  ble_agent_init();

  LOG_INF("--- Application is starting ---");

  while (1) {
    k_sleep(K_FOREVER);
  }

  return 0;
}

static void prvLedThread(void *arg0, void *arg1, void *arg2) {
  (void)arg0;
  (void)arg1;
  (void)arg2;

  bool led_state = true;

  while (1) {
    gpio_pin_toggle_dt(&led0);
    gpio_pin_toggle_dt(&led1);

    led_state = !led_state;
    k_msleep(SLEEP_TIME_MS);
  }
}

static void prvStatThread(void *arg0, void *arg1, void *arg2) {
  (void)arg0;
  (void)arg1;
  (void)arg2;

  k_thread_runtime_stats_t stats_thread;

  while (1) {
    k_sem_take(&btn3_press_sem, K_FOREVER);
    k_thread_runtime_stats_get(led_thread_id, &stats_thread);
    LOG_INF("Cycles of led thread: %ld", (long)stats_thread.execution_cycles);
  }
}

static void prvMpuThread(void *arg0, void *arg1, void *arg2) {
  (void)arg0;
  (void)arg1;
  (void)arg2;

  int16_t gx, gy, gz, ax, ay, az, t = 0;

  while (1) {
    mpu6050_read_acce(&ax, &ay, &az);
    mpu6050_read_gyro(&gx, &gy, &gz);
    mpu6050_read_temp(&t);

    LOG_INF("Acce: [x] %d - [y] %d - [z] %d", ax, ay, az);
    LOG_INF("Gyro: [x] %d - [y] %d - [z] %d", gx, gy, gz);
    LOG_INF("Temp: %d", t);

    // Sample data at 0.5Hz
    k_msleep(2000);
  }
}
