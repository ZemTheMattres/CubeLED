#set(component_srcs "led_strip_encoder.c")

idf_component_register(SRCS "led_strip_encoder.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver)

