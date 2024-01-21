set(srcs "main.c"
        "board.c")

idf_component_register(SRCS "sensors.c" "${srcs}"
                    INCLUDE_DIRS  ".")
