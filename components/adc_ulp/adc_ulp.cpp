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

// Reserve safe slots beyond instruction region
static const int BASELINE_SLOT  = 64;
static const int THRESHOLD_SLOT = 65;

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

void ulp_prog_run(uint32_t update_interval_ms, uint32_t adc_channel, uint16_t threshold) {

    const ulp_insn_t ulp_prog[] = {
        I_MOVI(R3, BASELINE_SLOT),   // R3 -> RTC_SLOW_MEM[BASELINE_SLOT]
        I_ADC(R0, adc_channel, 0),   // R0 = ADC(current)
        I_LD(R1, R3, 0),             // R1 = baseline
        I_SUBR(R2, R0, R1),          // R2 = difference = current - baseline
        I_MOVI(R3, THRESHOLD_SLOT),  // reuse R3 -> RTC_SLOW_MEM[THRESHOLD_SLOT]
        I_LD(R1, R3, 0),             // R1 = threshold (overwrites baseline)
        I_SUBR(R2, R2, R1),          // R2 = difference - threshold
        M_BGE(1, 0),                 // if R2 >= 0, goto label 1 (wake)
        M_BX(2),                     // otherwise, skip wake
        M_LABEL(1),
          I_WAKE(),                  // wake main CPU
          I_MOVI(R3, BASELINE_SLOT), // reload baseline slot address
          I_ST(R0, R3, 0),           // baseline = current
        M_LABEL(2),
        I_HALT()
    };

    // Microseconds to delay between halt and wake states
    ulp_set_wakeup_period(0, update_interval_ms * 1000);

    // Load and start ULP program
    size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_prog, &size);
    ulp_run(0);
}

void ADCULPSensor::setup() {

    // Stop any previously running ULP program
    ulp_timer_stop();

    // Init ADC unit
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    esp_err_t err_unit = adc_oneshot_new_unit(&init_cfg, &adc1_handle);
    if (err_unit != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing ADC1: %d", err_unit);
        this->mark_failed();
        return;
    }
    this->setup_flags_.handle_init_complete = true;

    // Init ADC channel 
    adc_oneshot_chan_cfg_t chan_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    esp_err_t err_channel = adc_oneshot_config_channel(adc1_handle, channel_, &chan_cfg);
    if (err_channel != ESP_OK) {
        ESP_LOGE(TAG, "Error configuring channel: %d", err_channel);
        this->mark_failed();
        return;
    }
    this->setup_flags_.config_complete = true;

    // Prime baseline with one CPU-side read
    int baseline = 0;
    esp_err_t err_read = adc_oneshot_read(adc1_handle, channel_, &baseline);
    if (err_read != ESP_OK) {
        ESP_LOGE(TAG, "Error reading channel: %d", err_read);
        this->mark_failed();
        return;
    }
    this->setup_flags_.calibration_complete = true;
    ESP_LOGI(TAG, "Primed baseline with initial ADC value: %d", baseline);

    // Initial values for the ULP memory
    RTC_SLOW_MEM[BASELINE_SLOT] = baseline;
    RTC_SLOW_MEM[THRESHOLD_SLOT] = threshold_;

    // Use internal led for testing
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    // Run ULP
    ulp_prog_run(update_interval_ms_, channel_, threshold_); 
    esp_sleep_enable_ulp_wakeup();

    this->setup_flags_.init_complete = true;
}

void ADCULPSensor::loop() {
    // if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP) {
    //     uint32_t measured = RTC_SLOW_MEM[BASELINE_SLOT];
    //     ESP_LOGI(TAG, "ULP wake-up, measured ADC = %u", measured);

    //     // Light the LED as a simple test
    //     gpio_set_level(GPIO_NUM_2, 1);  // ON
    //     delay(100);              // keep it on briefly
    //     gpio_set_level(GPIO_NUM_2, 0);  // OFF

    //     publish_state(measured);   // publish to YAML sensor
    // }

    static bool published = false;
    if (!published && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP) {
        uint32_t measured = RTC_SLOW_MEM[BASELINE_SLOT];
        publish_state(measured);
        published = true;
    }

    // uint32_t measured = RTC_SLOW_MEM[BASELINE_SLOT];
    // ESP_LOGI(TAG, "ULP measured ADC = %u", measured);

    // // Publish to YAML sensor
    // publish_state(measured);  

    // // Light the LED as a simple test
    // gpio_set_level(GPIO_NUM_2, 1);  // ON
    // delay(1000);              // keep it on briefly
    // gpio_set_level(GPIO_NUM_2, 0);  // OFF
    // delay(1000);              // keep it on briefly
}

void ADCULPSensor::dump_config() {
    LOG_SENSOR("", "ADC ULP Sensor", this);
    LOG_PIN("  Pin: ", this->pin_);
    ESP_LOGCONFIG(TAG,
                "  Channel:       %d\n"
                "  Unit:          ADC1\n"
                "  Attenuation:   %s\n",
                this->channel_, LOG_STR_ARG(attenuation_to_str(this->attenuation_)));

    ESP_LOGCONFIG(
        TAG,
        "  Setup Status:\n"
        "    Handle Init:  %s\n"
        "    Config:       %s\n"
        "    Calibration:  %s\n"
        "    Overall Init: %s",
        this->setup_flags_.handle_init_complete ? "OK" : "FAILED", this->setup_flags_.config_complete ? "OK" : "FAILED",
        this->setup_flags_.calibration_complete ? "OK" : "FAILED", this->setup_flags_.init_complete ? "OK" : "FAILED");
}

float ADCULPSensor::sample() {
    // TODO: Fix to return ULP value
    return 0;
}

}  // namespace adc_ulp
}  // namespace esphome

#endif  // USE_ESP32
