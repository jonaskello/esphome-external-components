#include "esphome/core/log.h"
#include "ulp_adc.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "esphome.h"
#include "esphome/core/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_sleep.h"

namespace esphome {
namespace ulp_adc {

static const char *TAG = "ulp_adc.component";

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

int gpio_to_adc_channel(int gpio) {
#if CONFIG_IDF_TARGET_ESP32
switch (gpio) {
    case 36: return 0;
    case 37: return 1;
    case 38: return 2;
    case 39: return 3;
    case 32: return 4;
    case 33: return 5;
    case 34: return 6;
    case 35: return 7;
    default: return -1;
}
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    switch (gpio) {
        case 1: return 0;
        case 2: return 1;
        case 3: return 2;
        case 4: return 3;
        case 5: return 4;
        case 6: return 5;
        case 7: return 6;
        case 8: return 7;
        case 9: return 8;
        case 10: return 9;
        default: return -1;
    }
#else
  return -1; // unsupported chip
#endif
}

void UlpAdc::setup() {

    // Stop any previously running ULP program
    ulp_timer_stop();

    // Convert GPIO pin to ADC channel
    int gpio = pin_->get_pin();
    adc_channel_ = gpio_to_adc_channel(gpio);            
    if (adc_channel_ < 0) {
        mark_failed("Specified pin is not ADC1-capable, only ADC1 can be used by ULP");
        return;
    }

    // Init ADC pin
    // adc1_config_width(ADC_WIDTH_BIT_12); // 12‑bit resolution
    // adc1_config_channel_atten((adc1_channel_t)adc_channel_, ADC_ATTEN_DB_12); // 11 dB attenuation equivalent
    // // --- New oneshot ADC setup ---
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));
    adc_oneshot_chan_cfg_t chan_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, (adc_channel_t)adc_channel_, &chan_cfg));

    // Prime baseline with one CPU-side read
    int baseline = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, (adc_channel_t)adc_channel_, &baseline));
    ESP_LOGI(TAG, "Primed baseline with initial ADC value: %d", baseline);

    // Initial values for the ULP memory
    RTC_SLOW_MEM[12] = baseline;
    RTC_SLOW_MEM[13] = threshold_;

    // Use internal led for testing
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    // Run ULP
    uint32_t delay_us = static_cast<uint32_t>(interval_) * 1000; // interval_ is in milliseconds
    ulp_adc_run(delay_us, adc_channel_, threshold_); 
}

void UlpAdc::loop() {
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP) {
        uint32_t measured = RTC_SLOW_MEM[12];
        ESP_LOGI(TAG, "ULP wake-up, measured ADC = %u", measured);

        // Light the LED as a simple test
        gpio_set_level(GPIO_NUM_2, 1);  // ON
        delay(100);              // keep it on briefly
        gpio_set_level(GPIO_NUM_2, 0);  // OFF

        publish_state(measured);   // publish to YAML sensor
    }
}


void UlpAdc::dump_config() {
    if(adc_channel_ < 0) {
        ESP_LOGE(TAG, "Pin %d is not ADC1-capable, only ADC1 can be used by ULP", pin_->get_pin());
        return;
    }
    ESP_LOGCONFIG(TAG, "UlpAdc:");
    ESP_LOGCONFIG(TAG, "  Interval: %u ms", interval_);
    LOG_PIN("  Pin: ", pin_);
    ESP_LOGCONFIG(TAG, "  ADC Channel is %d", adc_channel_);
}

}  // namespace ulp_adc
}  // namespace esphome
