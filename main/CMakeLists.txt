idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES myUsb myTcm myNvs wrc_common
    PRIV_REQUIRES esp_timer esp_psram led_strip driver
)
