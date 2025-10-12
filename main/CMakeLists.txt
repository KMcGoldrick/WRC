idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "." "../includes"
    REQUIRES usb myNvs myTcm myUsb
    PRIV_REQUIRES esp_timer esp_driver_gpio led_strip)
