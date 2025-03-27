#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "board_def.h"
#include "mpu6050.h"

LOG_MODULE_REGISTER(task_sensor);

// ZBus channels definition
ZBUS_CHAN_DEFINE(sensor_data,          // Name
                 struct mpu6050_data,  // Message type
                 NULL,                 // Validator
                 NULL,                 // User data
                 ZBUS_OBSERVERS_EMPTY, // Observers
                 ZBUS_MSG_INIT(0)      // Initial value
);

ZBUS_CHAN_DEFINE(trigger,                            // Name
                 enum TiggerSource,                  // Message type
                 NULL,                               // Validator
                 NULL,                               // User data
                 ZBUS_OBSERVERS(trigger_sub, blink), // Observers
                 0                                   // Initial value
);

// Sensor task configration
#define TASK_SENSOR_STACK_SIZE (1024)
#define TASK_SENSOR_PRIORITY (1)
static void prvTaskSensor(void *sub);
ZBUS_SUBSCRIBER_DEFINE(trigger_sub, 1);
K_THREAD_DEFINE(task_sensor_id, TASK_SENSOR_STACK_SIZE, prvTaskSensor,
                &trigger_sub, NULL, NULL, TASK_SENSOR_PRIORITY, K_ESSENTIAL, 0);

// Synchronization variables
K_SEM_DEFINE(sem_task_sensor_start, 0, 1);

// Interrupt structures and callbacks
static struct gpio_callback btn3_cb_data;

void prvBtn3Pressed(const struct device *port, struct gpio_callback *cb,
                    gpio_port_pins_t pins) {
  // Notify the trigger channel
  zbus_chan_notify(&trigger, K_NO_WAIT);
}

static void prvLed3Toggle(const struct zbus_channel *chan) {
  gpio_pin_toggle_dt(&led3);
}
ZBUS_LISTENER_DEFINE(blink, prvLed3Toggle);

bool task_sensor_init(void) {
  if (!mpu6050_sensor_init()) {
    LOG_ERR("Failed to initialize MPU-6050 sensor");
    return false;
  }

  gpio_init_callback(&btn3_cb_data, prvBtn3Pressed, BIT(btn3.pin));
  gpio_add_callback_dt(&btn3, &btn3_cb_data);
  gpio_pin_interrupt_configure_dt(&btn3, GPIO_INT_EDGE_FALLING);

  k_sem_give(&sem_task_sensor_start);
  return true;
}

static void prvTaskSensor(void *sub) {
  const struct zbus_channel *channel;
  const struct zbus_observer *trigger_sub = sub;
  struct mpu6050_data data = {0};
  int ret;

  k_sem_take(&sem_task_sensor_start, K_FOREVER);

  // Wait until we get notified by the trigger channel
  while (!zbus_sub_wait(trigger_sub, &channel, K_FOREVER)) {
    if (&trigger != channel) {
      LOG_ERR("Wrong channel triggered: %s (expected: %s)", channel->name,
              trigger.name);
      continue;
    }
    // Acquire data
    mpu6050_get_data(&data);

    // Publish data to the zbus chanel
    ret = zbus_chan_pub(&sensor_data, &data, K_SECONDS(1));
    if (ret != 0) {
      LOG_ERR("Unable to publish to the %s channel",
              zbus_chan_name(&sensor_data));
    }

    // Wait for 2 second before enable another acquisition
    k_sleep(K_SECONDS(2));
  }
}
