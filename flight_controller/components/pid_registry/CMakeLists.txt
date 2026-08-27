# pid_registry - live PID tuning and debug streaming for flight_control's eight PID instances.
# Sits between the cascade and the telemetry link so neither has to know about the other.
idf_component_register(SRCS "pid_registry.c"
                    INCLUDE_DIRS "include"
                    # PUBLIC: pid_registry.h exposes pid_controller_t, the PID payload structs and
                    # QueueHandle_t/TickType_t in its API, so every caller needs all three headers.
                    REQUIRES pid_controller telemetry_uart freertos
                    # PRIVATE: only the implementation timestamps samples.
                    PRIV_REQUIRES esp_timer)
