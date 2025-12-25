idf_component_register(
    SRCS "myTcm.c"
    INCLUDE_DIRS "."
    REQUIRES myUsb myNvs wrc_common myUart
)
