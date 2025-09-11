#ifndef WEB_UPLOAD_H
#define WEB_UPLOAD_H

#include "esp_http_server.h"

esp_err_t register_upload_handlers(httpd_handle_t server);
esp_err_t mass_erase_handler(httpd_req_t *req);  // Add this line
esp_err_t check_swd_handler(httpd_req_t *req);   // Add this line too

#endif