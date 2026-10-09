# An out-of-tree platform supplies its own drivers/platform/<name> from its own
# repository and adds it to the no-os target itself, so only descend here when
# the selected platform actually lives in this tree.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PLATFORM}")
    add_subdirectory(${PLATFORM})
endif()

if(${CONFIG_FREERTOS})
        add_subdirectory(freeRTOS)
endif()

no_os_sources_ifdef(CONFIG_PLATFORM_I2C_BITBANG ${CMAKE_CURRENT_SOURCE_DIR}/i2c/i2c_bitbang.c)
target_include_directories(no-os PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/i2c)
