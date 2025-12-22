idf_component_register(
    SRCS "myTcm.c"
    INCLUDE_DIRS "."
    REQUIRES  myUsb
    PRIV_REQUIRES main
    )