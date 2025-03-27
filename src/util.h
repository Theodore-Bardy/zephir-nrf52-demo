/**
 * @file util.h
 * @brief Definition of the util functions used in the demo
 */

#ifndef _UTIL_H_
#define _UTIL_H_

#ifdef CONFIG_ZTEST
#include <zephyr/ztest.h>
#define configASSERT(expression)                                               \
  do {                                                                         \
    int res = (expression);                                                    \
    if (0 == res) {                                                            \
      zassert_true(0);                                                         \
    }                                                                          \
  } while (0);
#elif defined(CONFIG_ASSERT)
#include <zephyr/sys/__assert.h>
#define configASSERT(expression) __ASSERT_NO_MSG(expression)
#else
#define configASSERT(expression)                                               \
  do {                                                                         \
    (void)sizeof(expression);                                                  \
  } while (0);
#endif

#endif // _UTIL_H_
