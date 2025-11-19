
cmake_minimum_required(VERSION 3.16)

project(esphome_rfm69 VERSION 1.0.0 LANGUAGES CXX)

# Встановлюємо C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Додаємо тестову директорію без завантаження ESPHome
add_subdirectory(test)