idf_component_register(
    SRCS "myUsb.c"
    INCLUDE_DIRS "."
    REQUIRES usb nvs_flash
    PRIV_REQUIRES main espressif__usb_host_cdc_acm esp_timer)