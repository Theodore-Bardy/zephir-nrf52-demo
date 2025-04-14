/**
 * @file app.c
 * @brief User Application loaded via LLEXT
 */

#include <zephyr/kernel.h>
#include <zephyr/llext/symbol.h>
// #include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

// LOG_MODULE_REGISTER(user_app);

static void prvHelloWorld(void) {
  // LOG_INF("Hello world from an llext!");
  printk("Hello world from an llext!\n");
  k_sleep(K_SECONDS(3));
}

void UserAppEntryPoint(void) {
  // User initialization

  // User code
  while (1) {
    prvHelloWorld();
  }
}
EXPORT_SYMBOL(UserAppEntryPoint);
