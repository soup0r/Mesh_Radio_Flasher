#ifndef WEB_UPLOAD_H
#define WEB_UPLOAD_H

#include "esp_http_server.h"

esp_err_t register_upload_handlers(httpd_handle_t server);
esp_err_t disable_protection_handler(httpd_req_t *req);
esp_err_t erase_all_handler(httpd_req_t *req);

#endif