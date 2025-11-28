#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ulp_blink {

class ULPBlink : public Component {
    public:
        void setup() override;
        void loop() override;
        void dump_config() override;

        void set_pin(GPIOPin *pin) { pin_ = pin; }
        void set_interval(uint32_t interval) { interval_ = interval; }

    protected:
        GPIOPin *pin_{nullptr};
        uint32_t interval_{1000};   // default 1s
        uint32_t last_toggle_{0};
        bool state_{false};
};

}  // namespace ulp_blink
}  // namespace esphome
