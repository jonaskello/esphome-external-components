#ifdef USE_ESP32

#include "adc_ulp.h"
#include "esphome/core/log.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "esphome.h"
#include "esphome/core/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_sleep.h"

namespace esphome {
namespace adc_ulp {

static const char *const TAG = "adc_ulp.esp32";

const LogString *attenuation_to_str(adc_atten_t attenuation) {
    switch (attenuation) {
        case ADC_ATTEN_DB_0:
            return LOG_STR("0 dB");
        case ADC_ATTEN_DB_2_5:
            return LOG_STR("2.5 dB");
        case ADC_ATTEN_DB_6:
            return LOG_STR("6 dB");
        case ADC_ATTEN_DB_12_COMPAT:
            return LOG_STR("12 dB");
        default:
            return LOG_STR("Unknown Attenuation");
    }
}

void ulp_adc_run(uint32_t us, uint32_t adc_channel, uint16_t threshold) {

    const ulp_insn_t ulp_adc_prog[] = {
        I_MOVI(R3, 12),              // R3 -> RTC_SLOW_MEM[12] (baseline slot)
        I_ADC(R0, adc_channel, 0),   // R0 = ADC(current)
        I_LD(R1, R3, 0),             // R1 = baseline
        I_SUBR(R2, R0, R1),          // R2 = difference = current - baseline
        I_MOVI(R3, 13),              // reuse R3 -> RTC_SLOW_MEM[13] (threshold slot)
        I_LD(R1, R3, 0),             // R1 = threshold (overwrites baseline)
        I_SUBR(R2, R2, R1),          // R2 = difference - threshold
        M_BGE(1, 0),                 // if R2 >= 0, goto label 1 (wake)
        M_BX(2),                     // otherwise, skip wake
        M_LABEL(1),
          I_WAKE(),                  // wake main CPU
          I_MOVI(R3, 12),            // reload baseline slot address
          I_ST(R0, R3, 0),           // baseline = current
        M_LABEL(2),
        I_HALT()
    };

    // Microseconds to delay between halt and wake states
    ulp_set_wakeup_period(0, us);

    // Load and start ULP program
    size_t size = sizeof(ulp_adc_prog) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_adc_prog, &size);
    ulp_run(0);
}

void ADCULPSensor::setup() {
    ESP_LOGCONFIG(TAG, "Initializing ULP ADC sensor");

    // Configure ULP program here (load binary, set channel, threshold, etc.)
    // Example placeholder:
    bool ulp_init_success = false;
    if (!ulp_init_success) {
        ESP_LOGE(TAG, "ULP init failed");
        this->mark_failed();
        return;
    }

    ESP_LOGCONFIG(TAG, "ULP ADC sensor init complete");
}

void ADCULPSensor::dump_config() {
    LOG_SENSOR("", "ADC ULP Sensor", this);
    LOG_PIN("  Pin: ", this->pin_);
    ESP_LOGCONFIG(TAG,
                "  Channel:       %d\n"
                "  Unit:          ADC1\n"
                "  Attenuation:   %s\n",
                this->channel_, LOG_STR_ARG(attenuation_to_str(this->attenuation_)));

    // ESP_LOGCONFIG(
    //     TAG,
    //     "  Setup Status:\n"
    //     "    Handle Init:  %s\n"
    //     "    Config:       %s\n"
    //     "    Calibration:  %s\n"
    //     "    Overall Init: %s",
    //     this->setup_flags_.handle_init_complete ? "OK" : "FAILED", this->setup_flags_.config_complete ? "OK" : "FAILED",
    //     this->setup_flags_.calibration_complete ? "OK" : "FAILED", this->setup_flags_.init_complete ? "OK" : "FAILED");
}

float ADCULPSensor::sample() {
    // TODO: Fix to return ULP value
    return 0;
}

}  // namespace adc_ulp
}  // namespace esphome

#endif  // USE_ESP32
