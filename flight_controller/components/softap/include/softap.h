#ifndef SOFTAP_H
#define SOFTAP_H

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize NVS, network interface and start WiFi SoftAP (does not start HTTP server) */
void softap_init(void);

/** Start the HTTP server and register handlers. Returns httpd handle. */
httpd_handle_t softap_start(void);

/** Stop the HTTP server. */
void softap_stop(httpd_handle_t server);

#ifdef __cplusplus
}
#endif

#endif /* SOFTAP_H */
