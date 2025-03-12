
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_def.h"
#include "zephyr/drivers/gpio.h"

LOG_MODULE_REGISTER(taks1);

// Task configration
#define TASK1_STACK_SIZE (256)
#define TASK1_PRIORITY (3)
static void prvTask1(void *, void *, void *);
K_THREAD_DEFINE(task1_id, TASK1_STACK_SIZE, prvTask1, NULL, NULL, NULL,
                TASK1_PRIORITY, K_ESSENTIAL, 0);

// Synchronization variables
K_SEM_DEFINE(sem_task1_start, 0, 1);

// Internal variables
static uint64_t amount_of_work = 30000;

// Interrupt structures and callbacks
static struct gpio_callback btn0_cb_data;
static struct gpio_callback btn1_cb_data;

void prvBtn0Pressed(const struct device *port, struct gpio_callback *cb,
                    gpio_port_pins_t pins) {
  if (amount_of_work < 100000u) {
    amount_of_work += 5000;
    LOG_INF("Increase amount of work %lld", amount_of_work);
  }
}

void prvBtn1Pressed(const struct device *port, struct gpio_callback *cb,
                    gpio_port_pins_t pins) {
  if (amount_of_work > 10000u) {
    amount_of_work -= 5000;
    LOG_INF("Decrease amount of work %lld", amount_of_work);
  }
}

// Internal functions
static void prvSimulateWork(void) {
  for (size_t i = 0; i < amount_of_work; i++)
    ;
}

static void prvTask1(void *a, void *b, void *c) {
  k_sem_take(&sem_task1_start, K_FOREVER);
  while (true) {
    LOG_INF("Task 1 is running...");

    gpio_pin_toggle_dt(&led0);
    prvSimulateWork();
    gpio_pin_toggle_dt(&led0);

    k_sleep(K_SECONDS(5));
  }
}

bool task1_init(void) {
  // Initialize buttons callbacks
  gpio_init_callback(&btn0_cb_data, prvBtn0Pressed, BIT(btn0.pin));
  gpio_init_callback(&btn1_cb_data, prvBtn1Pressed, BIT(btn1.pin));

  gpio_add_callback_dt(&btn0, &btn0_cb_data);
  gpio_add_callback_dt(&btn1, &btn1_cb_data);

  gpio_pin_interrupt_configure_dt(&btn0, GPIO_INT_EDGE_FALLING);
  gpio_pin_interrupt_configure_dt(&btn1, GPIO_INT_EDGE_FALLING);

  k_sem_give(&sem_task1_start);
  return true;
}
