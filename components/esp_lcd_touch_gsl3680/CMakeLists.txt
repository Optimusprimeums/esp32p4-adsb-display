idf_component_register(SRCS "esp_lcd_touch_gsl3680.c" "gsl_point_id.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "esp_lcd"
                       PRIV_REQUIRES "esp_driver_gpio" "esp_driver_i2c")
# Vendor algorithm blob has lots of K&R-style code that trips modern warnings
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable -Wno-implicit-fallthrough)
