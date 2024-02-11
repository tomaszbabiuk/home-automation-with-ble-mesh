set(srcs "main.c" "board.c" "mesh_app.c" "ux.c" "digital_input.c" "led_strip_encoder.c" "rgb_led.c" "sensors.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
