#pragma once
#ifdef USE_ESP32
  // esp_adc includes
#else
    #error "This ADC component only supports ESP32 (ESP-IDF)."
#endif

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"  // This defines ADC_CHANNEL_MAX

namespace esphome {
namespace adc_ulp {

// clang-format off
#if (ESP_IDF_VERSION_MAJOR == 5 && \
    ((ESP_IDF_VERSION_MINOR == 0 && ESP_IDF_VERSION_PATCH >= 5) || \
    (ESP_IDF_VERSION_MINOR == 1 && ESP_IDF_VERSION_PATCH >= 3) || \
    (ESP_IDF_VERSION_MINOR >= 2)) \
    )
// clang-format on
static const adc_atten_t ADC_ATTEN_DB_12_COMPAT = ADC_ATTEN_DB_12;
#else
static const adc_atten_t ADC_ATTEN_DB_12_COMPAT = ADC_ATTEN_DB_11;
#endif

class ADCULPSensor : public sensor::Sensor, public Component, public voltage_sampler::VoltageSampler {
    public:
        /// Set up the ADC sensor by initializing hardware and calibration parameters.
        /// This method is called once during device initialization.
        void setup() override;

        /// Output the configuration details of the ADC sensor for debugging purposes.
        /// This method is called during the ESPHome setup process to log the configuration.
        void dump_config() override;

        /// Set the GPIO pin to be used by the ADC sensor.
        /// @param pin Pointer to an InternalGPIOPin representing the ADC input pin.
        void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }

        /// Enable or disable the output of raw ADC values (unprocessed data).
        /// @param output_raw Boolean indicating whether to output raw ADC values (true) or processed values (false).
        void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }

        /// Perform a single ADC sampling operation and return the measured value.
        /// This function handles raw readings, calibration, and averaging as needed.
        /// @return The sampled value as a float.
        float sample() override;

        /// Set the ADC attenuation level to adjust the input voltage range.
        /// This determines how the ADC interprets input voltages, allowing for greater precision
        /// or the ability to measure higher voltages depending on the chosen attenuation level.
        /// @param attenuation The desired ADC attenuation level (e.g., ADC_ATTEN_DB_0, ADC_ATTEN_DB_11).
        void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }

        /// Configure the ADC to use a specific channel on a specific ADC unit.
        /// This sets the channel for single-shot or continuous ADC measurements.
        /// @param unit The ADC unit to use (ADC_UNIT_1 or ADC_UNIT_2).
        /// @param channel The ADC channel to configure, such as ADC_CHANNEL_0, ADC_CHANNEL_3, etc.
        void set_channel(adc_unit_t unit, adc_channel_t channel) {
            this->adc_unit_ = unit;
            this->channel_ = channel;
        }

        /// Set whether autoranging should be enabled for the ADC.
        /// Autoranging automatically adjusts the attenuation level to handle a wide range of input voltages.
        /// @param autorange Boolean indicating whether to enable autoranging.
        void set_autorange(bool autorange) { this->autorange_ = autorange; }

        /// The interval that the ULP will read ADC
        void set_update_interval(uint32_t interval_ms) { this->update_interval_ms_ = interval_ms; }

    protected:
        bool output_raw_{false};
        InternalGPIOPin *pin_;

        float sample_autorange_();
        float sample_fixed_attenuation_();
        bool autorange_{false};
        uint32_t update_interval_ms_{1000};  // default 1s
        adc_atten_t attenuation_{ADC_ATTEN_DB_0};
        adc_channel_t channel_{};
        adc_unit_t adc_unit_{};
        struct SetupFlags {
            uint8_t init_complete : 1;
            uint8_t config_complete : 1;
            uint8_t handle_init_complete : 1;
            uint8_t calibration_complete : 1;
            uint8_t reserved : 4;
        } setup_flags_{};

};

}  // namespace adc_ulp
}  // namespace esphome