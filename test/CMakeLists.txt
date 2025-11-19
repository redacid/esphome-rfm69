cmake_minimum_required(VERSION 3.16)

project(rfm69_test)

# Встановлюємо C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Шляхи до заголовкових файлів
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../components/rfm69
        ${CMAKE_CURRENT_SOURCE_DIR}/esphome_headers
)

# Створюємо виконуваний файл для тестування
add_executable(rfm69_test
        main.cpp
        ../components/rfm69/rfm69.cpp
)

# Додаємо компіляційні визначення
target_compile_definitions(rfm69_test PRIVATE
        ESPHOME_LOG_HAS_CONFIG
)