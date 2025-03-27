#ifndef _SERIALIZATION_H_
#define _SERIALIZATION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void ser_decoder_bool(const uint8_t *buf, size_t *index, bool *value);
void ser_decoder_uint8(const uint8_t *buf, size_t *index, uint8_t *value);
void ser_decoder_uint16(const uint8_t *buf, size_t *index, uint16_t *value);
void ser_decoder_uint24(const uint8_t *buf, size_t *index, uint32_t *value);
void ser_decoder_uint32(const uint8_t *buf, size_t *index, uint32_t *value);
void ser_decoder_uint64(const uint8_t *buf, size_t *index, uint64_t *value);
void ser_decoder_string(const uint8_t *buf, size_t *index, char **value);
void ser_decoder_data(const uint8_t *buf, size_t *index, void *data,
                      size_t data_len);

void ser_encoder_bool(uint8_t *buf, size_t *index, bool value);
void ser_encoder_uint8(uint8_t *buf, size_t *index, uint8_t value);
void ser_encoder_uint16(uint8_t *buf, size_t *index, uint16_t value);
void ser_encoder_uint24(uint8_t *buf, size_t *index, uint32_t value);
void ser_encoder_uint32(uint8_t *buf, size_t *index, uint32_t value);
void ser_encoder_uint64(uint8_t *buf, size_t *index, uint64_t value);
void ser_encoder_string(uint8_t *buf, size_t *index, char *value);
void ser_encoder_data(uint8_t *buf, size_t *index, const void *data,
                      size_t data_len);

#endif // !_SERIALIZATION_H_
