idf_component_register(
    SRCS "myUart.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer myUart myTcm
)
