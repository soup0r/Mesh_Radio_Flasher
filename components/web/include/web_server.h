#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include "esp_http_server.h"

// Initialize web server
esp_err_t web_server_init(void);
esp_err_t web_server_stop(void);

// Register handlers
esp_err_t register_swd_handlers(httpd_handle_t server);
esp_err_t register_flash_handlers(httpd_handle_t server);
esp_err_t register_system_handlers(httpd_handle_t server);

// WebSocket support for real-time updates
typedef void (*ws_callback_t)(const char *data, size_t len);
esp_err_t ws_send_to_all(const char *data, size_t len);
esp_err_t ws_register_callback(ws_callback_t callback);

// Progress reporting
void report_flash_progress(uint32_t current, uint32_t total, const char *operation);

#endif // WEB_SERVER_H