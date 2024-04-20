idf_component_register(SRCS "sx1278.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
