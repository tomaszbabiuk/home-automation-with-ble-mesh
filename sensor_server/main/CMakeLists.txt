set(srcs "main.c" "board.c" "mesh_app.c" "ux.c" "digital_input.c" "led_strip_encoder.c" "rgb_led.c")
set(sensors "sensors.c" "sgp30.c")
idf_component_register(SRCS  "${sensors}" "${srcs}"
                    INCLUDE_DIRS  ".")
