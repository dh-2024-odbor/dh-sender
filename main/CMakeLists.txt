idf_component_register(SRCS "bme.c" "lora.c" "main.c"
                    INCLUDE_DIRS "./include")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")