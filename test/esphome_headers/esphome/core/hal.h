#pragma once
#include <cstdint>
#include <memory>
#include <string>

namespace esphome {

    namespace gpio {
        enum Flags : uint8_t {
            FLAG_NONE = 0x00,
            FLAG_INPUT = 0x01,
            FLAG_OUTPUT = 0x02,
            FLAG_OPEN_DRAIN = 0x04,
            FLAG_PULLUP = 0x08,
            FLAG_PULLDOWN = 0x10,
            FLAG_INVERTED = 0x20,
        };
    }

    class GPIOPin {
    public:
        virtual ~GPIOPin() = default;
        virtual void setup() = 0;
        virtual void pin_mode(gpio::Flags flags) {}
        virtual bool digital_read() = 0;
        virtual void digital_write(bool value) = 0;
        virtual std::string dump_summary() const = 0;
    };

    // Timing functions
    uint32_t millis();
    void delay(uint32_t ms);
    uint32_t micros();
    void delayMicroseconds(uint32_t us);

} // namespace esphome