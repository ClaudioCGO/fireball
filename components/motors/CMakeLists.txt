idf_component_register(
    SRCS "src/motors.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio board
)