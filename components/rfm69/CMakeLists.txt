idf_component_register(
    SRCS "rfm69.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver spi_flash
)
