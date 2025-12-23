idf_component_register(
    SRCS "myUsb.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash myNvs espressif__usb_host_cdc_acm wrc_common myTcm
    PRIV_REQUIRES usb esp_timer esp_psram driver
)
