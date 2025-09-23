#ifndef WEB_BLE_H
#define WEB_BLE_H

#include "esp_http_server.h"

// Register all BLE-related HTTP handlers
esp_err_t register_ble_handlers(httpd_handle_t server);

#endif // WEB_BLE_H