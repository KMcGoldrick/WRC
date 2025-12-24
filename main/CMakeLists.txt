idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES myUsb myTcm myNvs wrc_common esp_psram
    PRIV_REQUIRES esp_timer led_strip driver
)
