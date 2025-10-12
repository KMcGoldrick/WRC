idf_component_register(
    SRCS "myNvs.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash
    PRIV_REQUIRES main)