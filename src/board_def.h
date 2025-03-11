

/**
 * @file board_def.h
 * @brief Definition of the hardware used in the demo
 */

#ifndef _BOARD_DEF_H_
#define _BOARD_DEF_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

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

#endif // _BOARD_DEF_H_
