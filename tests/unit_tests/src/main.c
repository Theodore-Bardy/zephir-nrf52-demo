#include <stdbool.h>
#include <stdint.h>
#include <zephyr/ztest.h>

#include "serialization.h"

ZTEST_SUITE(serialization_decode_test, NULL, NULL, NULL, NULL, NULL);

ZTEST(serialization_decode_test, test_decode_bool) {
  uint8_t buf[] = {0x01};
  size_t index = 0;
  bool value = true;

  ser_decoder_bool(buf, &index, &value);

  zassert_true(value, "Expected value to be true");
  zassert_equal(index, sizeof(uint8_t), "Expected index: 1 not %d", index);
}

ZTEST_EXPECT_FAIL(serialization_decode_test, test_null_buf_bool);
ZTEST(serialization_decode_test, test_null_buf_bool) {
  size_t index = 0;
  bool value = true;

  ser_decoder_bool(NULL, &index, &value);
}

ZTEST_EXPECT_FAIL(serialization_decode_test, test_null_index_bool);
ZTEST(serialization_decode_test, test_null_index_bool) {
  uint8_t buf[] = {0x01};
  bool value = true;

  ser_decoder_bool(buf, NULL, &value);
}

ZTEST_EXPECT_FAIL(serialization_decode_test, test_null_value_bool);
ZTEST(serialization_decode_test, test_null_value_bool) {
  uint8_t buf[] = {0x01};
  size_t index = 0;

  ser_decoder_bool(buf, &index, NULL);
}

// TODO: test other functions
