set(AXIS_CORE_SRCS
    src/version.c
    src/pid.c
    src/protocol.c
    src/config_table.c
    src/axis.c
)

if(ESP_PLATFORM)
    idf_component_register(SRCS ${AXIS_CORE_SRCS} INCLUDE_DIRS include)
else()
    add_library(axis_core STATIC ${AXIS_CORE_SRCS})
    target_include_directories(axis_core PUBLIC include)
    target_compile_features(axis_core PUBLIC c_std_11)
    target_compile_options(axis_core PRIVATE -Wall -Wextra -Werror -Wdouble-promotion)
    target_link_libraries(axis_core PUBLIC m)
endif()
