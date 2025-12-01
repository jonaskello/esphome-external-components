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

#define DATA_BASE_SLOT     64
#define BASELINE_OFFSET    0
#define ARM_OFFSET         1
#define DEBUG1_OFFSET      2
#define DEBUG2_OFFSET      3

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

void ADCULPSensor::setup() {

    // Disable WiFi at boot
    // esphome::wifi::global_wifi_component->disable();

    // Stop any previously running ULP program
    ulp_timer_stop();

    // If this is the first power on (not wakeup), then initialize the ULP 
    if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_ULP) {

        ESP_LOGI(TAG, "First power on, init ULP...");

        // Init ADC unit for use with ULP
        adc_oneshot_unit_handle_t adc1_handle;
        adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_FSM };
        esp_err_t err_unit = adc_oneshot_new_unit(&init_cfg, &adc1_handle);
        if (err_unit != ESP_OK) {
            ESP_LOGE(TAG, "Error initializing ADC1: %d", err_unit);
            this->mark_failed();
            return;
        }
        this->setup_flags_.handle_init_complete = true;

        // Init ADC channel for use with ULP
        adc_oneshot_chan_cfg_t chan_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
        esp_err_t err_channel = adc_oneshot_config_channel(adc1_handle, channel_, &chan_cfg);
        if (err_channel != ESP_OK) {
            ESP_LOGE(TAG, "Error configuring channel: %d", err_channel);
            this->mark_failed();
            return;
        }
        this->setup_flags_.config_complete = true;

        // Set baseline out of range baseline to trigger first value publish
        RTC_SLOW_MEM[DATA_BASE_SLOT + BASELINE_OFFSET] = 0x7FFF;
        // Prevent measurements when CPU is awake by setting ARM = 0
        RTC_SLOW_MEM[DATA_BASE_SLOT + ARM_OFFSET] = 0;
        RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG1_OFFSET] = 1;
        RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG2_OFFSET] = 2;

    }

    // Define ULP program
    const ulp_insn_t ulp_prog[] = {

        // Use R3 as data pointer in the whole program
        I_MOVI(R3, DATA_BASE_SLOT), // R3 = base data pointer

        // Check ARM flag so we don't measure values when the CPU is awake
        I_LD(R0, R3, ARM_OFFSET),  // R0 = ARM
        M_BL(2, 1),                // if ARM < 1 it is 0 (it can be 1 or 0) → branch to label 2 (skip)

        // Run real program when armed
        I_ADC(R2, 0, (uint32_t)channel_),  // R2 = raw ADC
        I_LD(R1, R3, BASELINE_OFFSET),     // R1 = baseline
        I_SUBR(R0, R2, R1),                // R0 = raw - baseline
        // I_ST(R1, R3, DEBUG1_OFFSET),  // DEBUG
        // I_ST(R2, R3, DEBUG2_OFFSET),  // DEBUG
        M_BGE(1, threshold_),              // if diff >= threshold goto wake
        I_SUBR(R0, R1, R2),                // R0 = baseline - raw
        M_BGE(1, threshold_),              // if diff >= threshold goto wake
        M_BX(2),                           // else skip wake
        M_LABEL(1),
            I_ST(R2, R3, BASELINE_OFFSET),  // update baseline with raw ADC
            I_MOVI(R0, 0),                  // R0 = 0 to clear ARM
            I_ST(R0, R3, ARM_OFFSET),       // clear ARM before we wake the cpu so we don't run while it is awake
            I_WAKE(),                       // wake CPU
        M_LABEL(2),
            I_HALT()                        // halt
    };

    // Microseconds to delay between ULP halt and wake states
    esp_err_t r = ulp_set_wakeup_period(0, 100 * 1000);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ulp_set_wakeup_period failed: %d", r);
        this->mark_failed();
        return;
    }
    // Load and start ULP program
    size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
    r = ulp_process_macros_and_load(0, ulp_prog, &size);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ulp_process_macros_and_load failed: %d", r);
        this->mark_failed();
        return;
    }
    r = ulp_run(0);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ulp_run failed: %d", r);
        this->mark_failed();
        return;
    }

    // Enable wakeup from ULP
    esp_sleep_enable_ulp_wakeup();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    ESP_LOGI(TAG, "First power on, init ULP completed...");


    this->setup_flags_.init_complete = true;
}

void ADCULPSensor::loop() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Wakeup cause: %d", cause);
    if(cause == ESP_SLEEP_WAKEUP_ULP) {
        // Publish only on wakeup from ULP
        uint32_t raw_measure = RTC_SLOW_MEM[DATA_BASE_SLOT + BASELINE_OFFSET];
        uint16_t actual_measure = raw_measure & 0x0FFF; // Only 12 bits are used
        publish_state(actual_measure);
        ESP_LOGI(TAG, "Published ADC value: %u", actual_measure);
    }

    ESP_LOGI(TAG, "BASELINE_OFFSET: %u", RTC_SLOW_MEM[DATA_BASE_SLOT + BASELINE_OFFSET] & 0xFFFF);
    ESP_LOGI(TAG, "DEBUG1_OFFSET: %u", RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG1_OFFSET] & 0xFFFF);
    ESP_LOGI(TAG, "DEBUG2_OFFSET: %u", RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG2_OFFSET] & 0xFFFF);

    // Immediately go back to deep sleep
    ESP_LOGI(TAG, "Entering deep sleep until next ULP wake...");
    delay(100);  // give logger time to flush
    RTC_SLOW_MEM[DATA_BASE_SLOT + ARM_OFFSET] = 1; // Tell ULP to start making measurements
    esp_deep_sleep_start();
}

void ADCULPSensor::dump_config() {
    LOG_SENSOR("", "ADC ULP Sensor", this);
    LOG_PIN("  Pin: ", this->pin_);
    ESP_LOGCONFIG(TAG,
                "  Channel:       %d\n"
                "  Unit:          ADC1\n"
                "  Threshold:     %d\n"
                "  Attenuation:   %s\n",
                this->channel_, this->threshold_, LOG_STR_ARG(attenuation_to_str(this->attenuation_)));

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
