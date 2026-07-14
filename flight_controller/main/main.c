#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu_driver.h"

static const char *TAG = "FC_MAIN";

void fc_task(void *args) { //semnatura unui task freeRTOS trebuie sa contine un param de tip void*
    ESP_LOGI(TAG, "Flight controll task started on core %d", xPortGetCoreID());
    //init imu, pwm
    esp_err_t error = imu_setup(); 
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "IMU setup failed: %s", esp_err_to_name(error));
        vTaskDelete(NULL);
        return;
    }
    imu_calibrate_gyro();

    const TickType_t xFreq = pdMS_TO_TICKS(1000); // 1ms -> 1kHz
    TickType_t xLastWakeTime = xTaskGetTickCount();


    imu_raw_data_t imu_raw_data;
    imu_physical_data_t imu_physical_data;
    for (;;) {
        error = imu_read_raw_data(&imu_raw_data);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "IMU read failed: %s", esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(10)); // wait a bit before retrying
            continue;
        }
        imu_convert_raw_to_physical(&imu_raw_data, &imu_physical_data);


        ESP_LOGI(TAG, "IMU Data - Acc: (%.4f, %.4f, %.4f), Gyro: (%.4f, %.4f, %.4f), Temp: %.4f",
        imu_physical_data.acc_x_g, imu_physical_data.acc_y_g, imu_physical_data.acc_z_g,
        imu_physical_data.gyro_x_dps, imu_physical_data.gyro_y_dps, imu_physical_data.gyro_z_dps,
        imu_physical_data.temp_C
        );
                 
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
