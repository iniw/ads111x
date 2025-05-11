idf_component_register(
    SRCS
        ads111x/ads111x.cpp
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_i2c
)
