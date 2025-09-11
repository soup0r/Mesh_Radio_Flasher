#ifndef HEX_PARSER_H
#define HEX_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// HEX record types
typedef enum {
    HEX_TYPE_DATA = 0x00,
    HEX_TYPE_EOF = 0x01,
    HEX_TYPE_EXT_SEG_ADDR = 0x02,
    HEX_TYPE_START_SEG_ADDR = 0x03,
    HEX_TYPE_EXT_LIN_ADDR = 0x04,
    HEX_TYPE_START_LIN_ADDR = 0x05
} hex_record_type_t;

// Single hex record
typedef struct {
    uint8_t byte_count;
    uint16_t address;
    uint8_t type;
    uint8_t data[255];
    uint8_t checksum;
} hex_record_t;

// Stream parser context
typedef struct hex_stream_parser hex_stream_parser_t;

// Callback for each parsed record
typedef void (*hex_record_callback_t)(hex_record_t *record, uint32_t abs_addr, void *ctx);

// Create stream parser
hex_stream_parser_t* hex_stream_create(hex_record_callback_t callback, void *user_ctx);

// Parse chunk of hex data
esp_err_t hex_stream_parse(hex_stream_parser_t *parser, const uint8_t *data, size_t len);

// Reset parser for new file
void hex_stream_reset(hex_stream_parser_t *parser);

// Free parser
void hex_stream_free(hex_stream_parser_t *parser);

// Get current extended address
uint32_t hex_stream_get_base_addr(hex_stream_parser_t *parser);

#endif