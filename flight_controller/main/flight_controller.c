
/* Simple ESP-IDF SoftAP + HTTP server
   Serves a "Hello World" page at http://192.168.4.1/ when connected to the AP.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "softap.h"

void app_main(void)
{
	ESP_LOGI("main", "Initializing SoftAP component");

	softap_init();
	httpd_handle_t server = softap_start();
	/* suppress unused variable warning when server isn't used further */
	(void) server;

	/* keep running */
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
//TODO: can t use both joysticks at the same time on mobile, figure out why