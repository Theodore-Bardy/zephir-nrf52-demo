
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_def.h"
#include "zephyr/drivers/gpio.h"

LOG_MODULE_REGISTER(taks2);

// Task configration
#define TASK2_STACK_SIZE (256)
#define TASK2_PRIORITY (3)
static void prvTask2(void *, void *, void *);
K_THREAD_DEFINE(task2_id, TASK2_STACK_SIZE, prvTask2, NULL, NULL, NULL,
                TASK2_PRIORITY, K_ESSENTIAL, 0);

// Synchronization variables
K_SEM_DEFINE(sem_task2_start, 0, 1);

// Internal variables
static uint64_t amount_of_work = 30000;

// Interrupt structures and callbacks
static struct gpio_callback btn2_cb_data;
static struct gpio_callback btn3_cb_data;

void prvBtn2Pressed(const struct device *port, struct gpio_callback *cb,
                    gpio_port_pins_t pins) {
  if (amount_of_work < 100000u) {
    amount_of_work += 5000;
    LOG_INF("Increase amount of work %lld", amount_of_work);
  }
}

void prvBtn3Pressed(const struct device *port, struct gpio_callback *cb,
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

static void prvTask2(void *a, void *b, void *c) {
  k_sem_take(&sem_task2_start, K_FOREVER);
  while (true) {
    LOG_INF("Task 2 is running...");

    gpio_pin_toggle_dt(&led1);
    prvSimulateWork();
    gpio_pin_toggle_dt(&led1);

    k_sleep(K_SECONDS(5));
  }
}

bool task2_init(void) {
  // Initialize buttons callbacks
  gpio_init_callback(&btn2_cb_data, prvBtn2Pressed, BIT(btn0.pin));
  gpio_init_callback(&btn3_cb_data, prvBtn3Pressed, BIT(btn1.pin));

  gpio_add_callback_dt(&btn2, &btn2_cb_data);
  gpio_add_callback_dt(&btn3, &btn3_cb_data);

  gpio_pin_interrupt_configure_dt(&btn2, GPIO_INT_EDGE_FALLING);
  gpio_pin_interrupt_configure_dt(&btn3, GPIO_INT_EDGE_FALLING);

  k_sem_give(&sem_task2_start);
  return true;
}
