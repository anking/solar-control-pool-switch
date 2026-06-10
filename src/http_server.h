#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "esp_err.h"
#include "esp_http_server.h"
#include <stddef.h>

esp_err_t http_server_start(void);
void http_server_stop(void);
httpd_handle_t http_server_get_handle(void);

void http_server_ws_broadcast_status(const char *json, size_t len);

#endif // HTTP_SERVER_H
