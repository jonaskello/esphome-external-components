#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ulp_adc {

class UlpAdc : public Component, public sensor::Sensor {
    public:
        void setup() override;
        void loop() override;
        void dump_config() override;

        void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
        void set_interval(uint32_t interval) { interval_ = interval; }
        void set_threshold(uint16_t threshold) { threshold_ = threshold; }
    
    protected:
        InternalGPIOPin *pin_{nullptr};
        uint32_t interval_{1000};   // How often the ULP will read ADC
        uint16_t threshold_{100};  // ADC difference threshold for wake-up
        int adc_channel_{-1};   // store computed channel here
};

}  // namespace ulp_adc
}  // namespace esphome
