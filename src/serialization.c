#include <string.h>

#include "serialization.h"
#include "util.h"

void ser_decoder_bool(const uint8_t *buf, size_t *index, bool *value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Decode value */
  *value = (0x00 != buf[*index]) ? true : false;
  *index += sizeof(uint8_t);
}

void ser_decoder_uint8(const uint8_t *buf, size_t *index, uint8_t *value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Decode value */
  *value = buf[*index];
  *index += sizeof(uint8_t);
}

void ser_decoder_uint16(const uint8_t *buf, size_t *index, uint16_t *value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Decode value */
  *value =
      (uint16_t)((uint16_t)(buf[*index]) | (((uint16_t)buf[*index + 1]) << 8));
  *index += sizeof(uint16_t);
}

void ser_decoder_uint24(const uint8_t *buf, size_t *index, uint32_t *value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Decode value */
  *value = (uint32_t)(buf[*index]) | (((uint32_t)buf[*index + 1]) << 8) |
           (((uint32_t)buf[*index + 2]) << 16);
  *index += sizeof(uint16_t) + sizeof(uint8_t);
}

void ser_decoder_uint32(const uint8_t *buf, size_t *index, uint32_t *value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Decode value */
  *value = (uint32_t)(buf[*index]) | (((uint32_t)buf[*index + 1]) << 8) |
           (((uint32_t)buf[*index + 2]) << 16) |
           (((uint32_t)buf[*index + 3]) << 24);
  *index += sizeof(uint32_t);
}

void ser_decoder_uint64(const uint8_t *buf, size_t *index, uint64_t *value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Decode value */
  *value = (uint64_t)(buf[*index]) | (((uint64_t)buf[*index + 1]) << 8) |
           (((uint64_t)buf[*index + 2]) << 16) |
           (((uint64_t)buf[*index + 3]) << 24) |
           (((uint64_t)buf[*index + 4]) << 32) |
           (((uint64_t)buf[*index + 5]) << 40) |
           (((uint64_t)buf[*index + 6]) << 48) |
           (((uint64_t)buf[*index + 7]) << 56);
  *index += sizeof(uint64_t);
}

void ser_decoder_string(const uint8_t *buf, size_t *index, char **value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Decode value */
  *value = (char *)buf + *index;
  *index += strlen(*value) + 1;
}

void ser_decoder_data(const uint8_t *buf, size_t *index, void *data,
                      size_t data_len) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != data);

  /* Decode data */
  memcpy(data, &buf[*index], data_len);
  *index += data_len;
}

void ser_encoder_bool(uint8_t *buf, size_t *index, bool value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);

  /* Encode value */
  buf[*index] = (true == value) ? 0x01 : 0x00;
  *index += sizeof(uint8_t);
}

void ser_encoder_uint8(uint8_t *buf, size_t *index, uint8_t value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);

  /* Encode value */
  buf[*index] = value;
  *index += sizeof(uint8_t);
}

void ser_encoder_uint16(uint8_t *buf, size_t *index, uint16_t value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);

  /* Encode value */
  buf[*index] = (uint8_t)((value & 0x00FF) >> 0);
  buf[*index + 1] = (uint8_t)((value & 0xFF00) >> 8);
  *index += sizeof(uint16_t);
}

void ser_encoder_uint24(uint8_t *buf, size_t *index, uint32_t value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);

  /* Encode value */
  buf[*index] = (uint8_t)((value & 0x000000FF) >> 0);
  buf[*index + 1] = (uint8_t)((value & 0x0000FF00) >> 8);
  buf[*index + 2] = (uint8_t)((value & 0x00FF0000) >> 16);
  *index += sizeof(uint16_t) + sizeof(uint8_t);
}

void ser_encoder_uint32(uint8_t *buf, size_t *index, uint32_t value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);

  /* Encode value */
  buf[*index] = (uint8_t)((value & 0x000000FF) >> 0);
  buf[*index + 1] = (uint8_t)((value & 0x0000FF00) >> 8);
  buf[*index + 2] = (uint8_t)((value & 0x00FF0000) >> 16);
  buf[*index + 3] = (uint8_t)((value & 0xFF000000) >> 24);
  *index += sizeof(uint32_t);
}

void ser_encoder_uint64(uint8_t *buf, size_t *index, uint64_t value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);

  /* Encode value */
  buf[*index] = (uint8_t)((value & 0x00000000000000FF) >> 0);
  buf[*index + 1] = (uint8_t)((value & 0x000000000000FF00) >> 8);
  buf[*index + 2] = (uint8_t)((value & 0x0000000000FF0000) >> 16);
  buf[*index + 3] = (uint8_t)((value & 0x00000000FF000000) >> 24);
  buf[*index + 4] = (uint8_t)((value & 0x000000FF00000000) >> 32);
  buf[*index + 5] = (uint8_t)((value & 0x0000FF0000000000) >> 40);
  buf[*index + 6] = (uint8_t)((value & 0x00FF000000000000) >> 48);
  buf[*index + 7] = (uint8_t)((value & 0xFF00000000000000) >> 56);
  *index += sizeof(uint64_t);
}

void ser_encoder_string(uint8_t *buf, size_t *index, char *value) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != value);

  /* Encode value */
  strcpy((char *)buf + *index, value);
  size_t len = strlen(value);
  buf[*index + len] = 0;
  *index += len + 1;
}

void ser_encoder_data(uint8_t *buf, size_t *index, const void *data,
                      size_t data_len) {
  /* Check parameters */
  configASSERT(NULL != buf);
  configASSERT(NULL != index);
  configASSERT(NULL != data);

  /* Encode value */
  memcpy(&buf[*index], data, data_len);
  *index += data_len;
}
