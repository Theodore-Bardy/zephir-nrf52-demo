/**
 * @file main.c
 * @brief Zephyr Demo Application for the nRF52 DK Board
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(main);

#define SLEEP_TIME_MS (10000)

/**
 * @brief Hardware used in the demo
 */
#define LED0_NODE DT_NODELABEL(led0)
#define LED1_NODE DT_NODELABEL(led1)
#define LED2_NODE DT_NODELABEL(led2)
#define LED3_NODE DT_NODELABEL(led3)
#define BTN0_NODE DT_NODELABEL(button0)
#define BTN1_NODE DT_NODELABEL(button1)
#define BTN2_NODE DT_NODELABEL(button2)
#define BTN3_NODE DT_NODELABEL(button3)

/**
 * @brief Synchronization variables
 */
static struct k_sem btn1_press_sem;

/**
 * @brief Thread macro and variables
 */
#define LED_THREAD_STACK_SIZE (250)
#define LED_THREAD_PRIORITY   (3)
K_THREAD_STACK_DEFINE(led_thread_area, LED_THREAD_STACK_SIZE);
static k_tid_t led_thread_id;
static struct k_thread led_thread_cxt;
static void prvLedThread(void* arg0, void* arg1, void* arg2);

#define STAT_THREAD_STACK_SIZE (250)
#define STAT_THREAD_PRIORITY   (2)
K_THREAD_STACK_DEFINE(stat_thread_area, STAT_THREAD_STACK_SIZE);
static k_tid_t stat_thread_id;
static struct k_thread stat_thread_cxt;
static void prvStatThread(void* arg0, void* arg1, void* arg2);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);
static const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET(BTN0_NODE, gpios);
static const struct gpio_dt_spec btn1 = GPIO_DT_SPEC_GET(BTN1_NODE, gpios);
static const struct gpio_dt_spec btn2 = GPIO_DT_SPEC_GET(BTN2_NODE, gpios);
static const struct gpio_dt_spec btn3 = GPIO_DT_SPEC_GET(BTN3_NODE, gpios);

/**
 * @brief Interrupt structures and callbacks
 */
static struct gpio_callback btn0_cb_data;
static struct gpio_callback btn1_cb_data;
static struct gpio_callback btn2_cb_data;
static struct gpio_callback btn3_cb_data;

void btn0_pressed(const struct device* port, struct gpio_callback* cb, gpio_port_pins_t pins)
{
    // Toggle led2 when the btn0 is pressed
    gpio_pin_toggle_dt(&led2);
}

void btn1_pressed(const struct device* port, struct gpio_callback* cb, gpio_port_pins_t pins)
{
    gpio_pin_toggle_dt(&led3);
}

void btn2_pressed(const struct device* port, struct gpio_callback* cb, gpio_port_pins_t pins)
{
    LOG_INF("Monstre!");
}

void btn3_pressed(const struct device* port, struct gpio_callback* cb, gpio_port_pins_t pins)
{
    // Give semaphore when the btn1 is pressed
    k_sem_give(&btn1_press_sem);
}

int main(void)
{
    // Initialize hardware
    uint8_t ret;

    if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1) ||
        !gpio_is_ready_dt(&btn0) || !gpio_is_ready_dt(&btn1) ||
        !gpio_is_ready_dt(&btn2) || !gpio_is_ready_dt(&btn3))
    {
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
    k_sem_init(&btn1_press_sem, 0u, 1u);

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
    led_thread_id = k_thread_create(&led_thread_cxt,
                                    led_thread_area,
                                    K_THREAD_STACK_SIZEOF(led_thread_area),
                                    prvLedThread,
                                    NULL,
                                    NULL,
                                    NULL,
                                    LED_THREAD_PRIORITY,
                                    K_ESSENTIAL,
                                    K_NO_WAIT);

    stat_thread_id = k_thread_create(&stat_thread_cxt,
                                     stat_thread_area,
                                     K_THREAD_STACK_SIZEOF(stat_thread_area),
                                     prvStatThread,
                                     NULL,
                                     NULL,
                                     NULL,
                                     STAT_THREAD_PRIORITY,
                                     K_ESSENTIAL,
                                     K_NO_WAIT);

    LOG_INF("--- Application is starting ---");

    while (1)
    {
        k_sleep(K_FOREVER);
    }

  return 0;
}

static void
prvLedThread(void* arg0, void* arg1, void* arg2)
{
    (void)arg0;
    (void)arg1;
    (void)arg2;

    bool led_state = true;

    while (1) {
        gpio_pin_toggle_dt(&led0);
      gpio_pin_toggle_dt(&led1);

        led_state = !led_state;
        LOG_DBG("LED 0 state: %s", led_state ? "ON" : "OFF");
        LOG_DBG("LED 1 state: %s", led_state ? "OFF" : "ON");

        k_msleep(SLEEP_TIME_MS);
    }
}

static void
prvStatThread(void* arg0, void* arg1, void* arg2)
{
    (void)arg0;
    (void)arg1;
    (void)arg2;

    k_thread_runtime_stats_t stats_thread;

    while (1)
    {
        k_sem_take(&btn1_press_sem, K_FOREVER);
        k_thread_runtime_stats_get(led_thread_id, &stats_thread);
        LOG_INF("Cycles of led thread: %ld", (long)stats_thread.execution_cycles);
    }
}
