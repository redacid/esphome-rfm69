#pragma once
#include <cstdint>

namespace esphome {

    namespace setup_priority {
        static constexpr float HARDWARE = 1000.0f;
        static constexpr float DATA = 600.0f;
        static constexpr float LATE = 100.0f;
        static constexpr float AFTER_CONNECTION = -100.0f;
    }

    class Component {
    public:
        virtual ~Component() = default;
        virtual void setup() {}
        virtual void loop() {}
        virtual void dump_config() {}
        virtual float get_setup_priority() const { return setup_priority::DATA; }

    protected:
        void mark_failed() {}
    };

} // namespace esphome