#pragma once
#include "esphome/core/component.h"

namespace esphome {
    namespace spi {

        class SPIComponent : public Component {
        public:
            void setup() override {}
            void dump_config() override {}
        };

    } // namespace spi
} // namespace esphome

// Mock SPI object
extern class MockSPI {
public:
    void begin();
    uint8_t transfer(uint8_t data);
} SPI;