idf_component_register(
    SRCS "src/pwm.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc
)
