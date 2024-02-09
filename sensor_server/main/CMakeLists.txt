set(srcs "main.c"
        "board.c")

idf_component_register(SRCS "ble_mesh.c" "ux.c" "digital_input.c" "led_strip_encoder.c" "rgb_led.c" "sensors.c" "${srcs}"
                    INCLUDE_DIRS  ".")
