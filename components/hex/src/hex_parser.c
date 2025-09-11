#include "hex_parser.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "HEX_PARSER";

struct hex_stream_parser {
    uint32_t extended_addr;
    uint32_t segment_addr;
    uint8_t line_buffer[600];
    int line_pos;
    hex_record_callback_t callback;
    void *user_ctx;
    uint32_t line_count;
    uint32_t data_bytes;
};

static uint8_t hex_char_to_byte(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

static uint8_t hex_to_byte(const char *hex) {
    return (hex_char_to_byte(hex[0]) << 4) | hex_char_to_byte(hex[1]);
}

static esp_err_t parse_hex_line(const char *line, hex_record_t *record) {
    if (line[0] != ':') {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Minimum line length check
    if (strlen(line) < 11) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Parse byte count
    record->byte_count = hex_to_byte(line + 1);
    
    // Parse address
    record->address = (hex_to_byte(line + 3) << 8) | hex_to_byte(line + 5);
    
    // Parse type
    record->type = hex_to_byte(line + 7);
    
    // Validate line length
    size_t expected_len = 11 + (record->byte_count * 2);
    if (strlen(line) < expected_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Parse data
    for (int i = 0; i < record->byte_count; i++) {
        record->data[i] = hex_to_byte(line + 9 + (i * 2));
    }
    
    // Parse and verify checksum
    record->checksum = hex_to_byte(line + 9 + (record->byte_count * 2));
    
    // Calculate checksum
    uint8_t sum = record->byte_count + 
                  (record->address >> 8) + 
                  (record->address & 0xFF) + 
                  record->type;
    
    for (int i = 0; i < record->byte_count; i++) {
        sum += record->data[i];
    }
    
    sum = (~sum) + 1;
    
    if (sum != record->checksum) {
        ESP_LOGE(TAG, "Checksum mismatch: calculated 0x%02X, got 0x%02X", 
                 sum, record->checksum);
        return ESP_ERR_INVALID_CRC;
    }
    
    return ESP_OK;
}

hex_stream_parser_t* hex_stream_create(hex_record_callback_t callback, void *user_ctx) {
    hex_stream_parser_t *parser = calloc(1, sizeof(hex_stream_parser_t));
    if (parser) {
        parser->callback = callback;
        parser->user_ctx = user_ctx;
    }
    return parser;
}

esp_err_t hex_stream_parse(hex_stream_parser_t *parser, const uint8_t *data, size_t len) {
    if (!parser) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (size_t i = 0; i < len; i++) {
        char c = data[i];
        
        if (c == '\n' || c == '\r') {
            if (parser->line_pos > 0) {
                parser->line_buffer[parser->line_pos] = '\0';
                
                hex_record_t record;
                esp_err_t ret = parse_hex_line((char*)parser->line_buffer, &record);
                
                if (ret == ESP_OK) {
                    parser->line_count++;
                    
                    // Handle different record types
                    switch (record.type) {
                        case HEX_TYPE_DATA:
                            parser->data_bytes += record.byte_count;
                            if (parser->callback) {
                                uint32_t abs_addr = record.address + 
                                                   parser->extended_addr + 
                                                   parser->segment_addr;
                                parser->callback(&record, abs_addr, parser->user_ctx);
                            }
                            break;
                            
                        case HEX_TYPE_EOF:
                            ESP_LOGI(TAG, "EOF record found. Lines: %lu, Data bytes: %lu",
                                    parser->line_count, parser->data_bytes);
                            if (parser->callback) {
                                parser->callback(&record, 0, parser->user_ctx);
                            }
                            break;
                            
                        case HEX_TYPE_EXT_LIN_ADDR:
                            parser->extended_addr = ((uint32_t)record.data[0] << 24) | 
                                                   ((uint32_t)record.data[1] << 16);
                            ESP_LOGI(TAG, "Extended linear address: 0x%08lX", 
                                    parser->extended_addr);
                            break;
                            
                        case HEX_TYPE_EXT_SEG_ADDR:
                            parser->segment_addr = (((uint32_t)record.data[0] << 8) | 
                                                   record.data[1]) << 4;
                            ESP_LOGI(TAG, "Extended segment address: 0x%08lX", 
                                    parser->segment_addr);
                            break;
                            
                        case HEX_TYPE_START_LIN_ADDR:
                            ESP_LOGI(TAG, "Start linear address (entry point) ignored");
                            break;
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to parse line %lu: %s", 
                            parser->line_count + 1, (char*)parser->line_buffer);
                }
                
                parser->line_pos = 0;
            }
        } else if (parser->line_pos < sizeof(parser->line_buffer) - 1) {
            parser->line_buffer[parser->line_pos++] = c;
        } else {
            ESP_LOGE(TAG, "Line too long");
            parser->line_pos = 0;
        }
    }
    
    return ESP_OK;
}

void hex_stream_reset(hex_stream_parser_t *parser) {
    if (parser) {
        parser->extended_addr = 0;
        parser->segment_addr = 0;
        parser->line_pos = 0;
        parser->line_count = 0;
        parser->data_bytes = 0;
    }
}

void hex_stream_free(hex_stream_parser_t *parser) {
    free(parser);
}

uint32_t hex_stream_get_base_addr(hex_stream_parser_t *parser) {
    return parser ? (parser->extended_addr + parser->segment_addr) : 0;
}