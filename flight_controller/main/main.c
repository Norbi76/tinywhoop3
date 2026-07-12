#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu_driver.h"

static const char *TAG = "FC_MAIN";

void fc_task(void *args) { //semnatura unui task freeRTOS trebuie sa contine un param de tip void*
    ESP_LOGI(TAG, "Flight controll task started on core %d", xPortGetCoreID());
    //init imu, pwm
    imu_setup(); //needs error handling

    const TickType_t xFreq = pdMS_TO_TICKS(1); // 1ms -> 1kHz
    TickType_t xLastWakeTime = xTaskGetTickCount();


    for (;;) {
        // 1. Citește giroscop / accelerometru
        // 2. Calculează PID
        // 3. Scrie comanda PWM către motoarele coreless

        // Așteaptă exact până la următoarea milisecundă
        vTaskDelayUntil(&xLastWakeTime, xFreq);
    }
}

void telemetry_rx_task(void *args) {
    //init uart...
    ESP_LOGI(TAG, "Telemtry transmission task started on core %d", xPortGetCoreID());

    for (;;) {

        vTaskDelay(pdMS_TO_TICKS(20));
    }

}

void app_main(void)
{
    ESP_LOGI(TAG, "System starting up...");
    
    xTaskCreatePinnedToCore(
        fc_task,
        "Flight_Ctrl_Task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1, //prioritatea cea mai mare, pentru a nu fi intrerupt de alte taskuri
        NULL,
        1
    );
    //de ce am ales core 1 pt task ul FC?
    //Core 0 -> PRO_CPU(PROTOCOL CPU): ruleaza multe task uri de "fundal" cum ar fi accese la mem, alte functii legate de os
    //Core 1 -> APP_CPU(APPLICATION CPU): este lasat mai liber

    xTaskCreatePinnedToCore(
        telemetry_rx_task,
        "Telemetry_Task",
        4096,
        NULL,
        5,
        NULL,
        0
    );
}
