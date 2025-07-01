
idf_component_register(
        SRCS "main.c" "gatt_dice.c"
        INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}")

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/gatt_dice.h
            COMMAND $ENV{IDF_PATH}/components/btstack/tool/compile_gatt.py ${COMPONENT_DIR}/gatt_dice.gatt ${CMAKE_CURRENT_BINARY_DIR}/gatt_dice.h
            DEPENDS gatt_dice.gatt
            VERBATIM
    )
    add_custom_target(GATT_DB DEPENDS gatt_dice.h)
    add_dependencies(${COMPONENT_LIB} GATT_DB)
endif()
