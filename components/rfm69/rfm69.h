#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <SPI.h>

namespace esphome {
    namespace rfm69 {

        class RFM69Component : public Component {
        public:
            void setup() override;
            void loop() override;
            void dump_config() override;

            void set_cs_pin(GPIOPin *cs_pin) { cs_pin_ = cs_pin; }
            void set_irq_pin(GPIOPin *irq_pin) { irq_pin_ = irq_pin; }
            void set_rst_pin(GPIOPin *rst_pin) { rst_pin_ = rst_pin; }
            void set_frequency(float frequency) { frequency_ = frequency; }
            void set_network_id(uint8_t network_id) { network_id_ = network_id; }
            void set_node_id(uint8_t node_id) { node_id_ = node_id; }
            void set_high_power(bool high_power) { high_power_ = high_power; }
            void set_power_level(uint8_t power_level) { power_level_ = power_level; }

            bool send_message(uint8_t to_address, const std::string &data);

        protected:
            GPIOPin *cs_pin_{nullptr};
            GPIOPin *irq_pin_{nullptr};
            GPIOPin *rst_pin_{nullptr};
            float frequency_{868.0};
            uint8_t network_id_{100};
            uint8_t node_id_{1};
            bool high_power_{true};
            uint8_t power_level_{20};

            bool initialize_();
            void write_register_(uint8_t addr, uint8_t value);
            uint8_t read_register_(uint8_t addr);
        };

    }  // namespace rfm69
}  // namespace esphome