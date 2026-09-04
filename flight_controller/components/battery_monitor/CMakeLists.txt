# battery_monitor - pack voltage from a resistor divider on one ADC pin.
# Leaf component: flight_control depends on it, it depends on nothing in this repo.
idf_component_register(SRCS "battery_monitor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_adc)
