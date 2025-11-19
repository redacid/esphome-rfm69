#cmake_minimum_required(VERSION 3.10)
#
## Визначаємо назву компонента
#set(COMPONENT_NAME "rfm69")
#
## Знаходимо ESPHome
#find_path(ESPHOME_PATH
#        NAMES components
#        PATHS
#        ${CMAKE_SOURCE_DIR}/../../..
#        ${CMAKE_SOURCE_DIR}/../../../esphome
#        NO_DEFAULT_PATH
#)
#
#if(NOT ESPHOME_PATH)
#    message(FATAL_ERROR "Could not find ESPHome path")
#endif()
#
## Створюємо бібліотеку компонента
#add_library(${COMPONENT_NAME}
#        rfm69.cpp
#        rfm69.h
#        rfm69_registers.h
#)
#
## Додаємо включення директорій
#target_include_directories(${COMPONENT_NAME} PUBLIC
#        ${CMAKE_CURRENT_SOURCE_DIR}
#        ${ESPHOME_PATH}
#        ${ESPHOME_PATH}/esphome/core
#        ${ESPHOME_PATH}/esphome/components
#)
#
## Лінкуємо з ESPHome компонентами
#target_link_libraries(${COMPONENT_NAME} PUBLIC
#        esphome_core
#        esphome_spi
#)
#
## Компілятор флаги
#target_compile_definitions(${COMPONENT_NAME} PRIVATE
#        USE_ESP32
#        ESPHOME_LOG_HAS_CONFIG
#)
#
#set_property(TARGET ${COMPONENT_NAME} PROPERTY CXX_STANDARD 17)