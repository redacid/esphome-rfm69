#pragma once
#include <cstdio>
#include <cstdarg>

namespace esphome {

    // Log levels
#define ESPHOME_LOG_LEVEL_NONE 0
#define ESPHOME_LOG_LEVEL_ERROR 1
#define ESPHOME_LOG_LEVEL_WARN 2
#define ESPHOME_LOG_LEVEL_INFO 3
#define ESPHOME_LOG_LEVEL_CONFIG 4
#define ESPHOME_LOG_LEVEL_DEBUG 5
#define ESPHOME_LOG_LEVEL_VERBOSE 6

    void esp_log_printf_(int level, const char *tag, int line, const char *format, ...);
    void esp_log_set_level(const char* tag, int level);

} // namespace esphome

// Logging macros
#define ESP_LOGE(tag, format, ...) esphome::esp_log_printf_(1, tag, __LINE__, format, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) esphome::esp_log_printf_(2, tag, __LINE__, format, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) esphome::esp_log_printf_(3, tag, __LINE__, format, ##__VA_ARGS__)
#define ESP_LOGCONFIG(tag, format, ...) esphome::esp_log_printf_(4, tag, __LINE__, format, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) esphome::esp_log_printf_(5, tag, __LINE__, format, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) esphome::esp_log_printf_(6, tag, __LINE__, format, ##__VA_ARGS__)