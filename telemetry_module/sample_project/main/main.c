#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "Telemetry Module";

void telemetry_task(void *pvParameters) {
    ESP_LOGI("Telemetry Task", "Telemetry task started");

    for (;;) {
        // ==========================================
        // 1. ZONA DE RECEPȚIE (RX)
        // Aici vom citi constant magistrala pentru a 
        // prelua unghiurile de zbor de la Controler.
        // ==========================================
        
        // TODO: Apelare funcție de citire UART și validare pachet

        // ==========================================
        // 2. ZONA DE TRANSMISIE (TX)
        // Aici vom verifica dacă avem comenzi noi 
        // (ex: de la aplicația Wi-Fi) pe care trebuie 
        // să le trimitem înapoi către Controler.
        // ==========================================

        // TODO: Verificare mesaje în așteptare și scriere pe UART

        // ==========================================
        // 3. PREDAREA CONTROLULUI (YIELD)
        // ==========================================
        // Lasă procesorul să respire pentru a executa 
        // și alte task-uri (Wi-Fi, procesare video).
        // 10ms = frecvență de rulare de ~100 Hz.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Telemtry module starting up...");

    xTaskCreatePinnedToCore(
        telemetry_task,       // Funcția task-ului
        "TELEMETRY_TASK",     // Nume intern (pentru debug)
        4096,                 // Memorie alocată (Stivă)
        NULL,                 // Parametri
        5,                    // Prioritate (5 e o prioritate medie-mare)
        NULL,                 // Handle
        0                     // Alocat pe Core 1
    );
}   
