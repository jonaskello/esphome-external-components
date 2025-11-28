#include "esphome/core/log.h"
#include "ulp_blink.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "soc/rtc_io_reg.h"
#include "esphome.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace ulp_blink {

static const char *TAG = "ulp_blink.component";

void ULP_BLINK_RUN(uint32_t us, uint32_t bit);

void ULPBlink::setup() {

    // Stop any previously running ULP program
    ulp_timer_stop();

    // Convert GPIO pin to RTC IO index
    gpio_num_t gpio = (gpio_num_t) pin_->get_pin();
    int rtc_num = rtc_io_number_get(gpio);            
    if (rtc_num < 0) {
        mark_failed("Specified pin is not RTC-capable");
        return;
    }
    rtc_bit_ = RTC_GPIO_OUT_DATA_S + rtc_num;

    // Init LED pin
    rtc_gpio_init(gpio);
    rtc_gpio_set_direction(gpio, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(gpio, 0);

    // Run ULP
    int delay_us = interval_ * 1000; // interval_ is in ms
    ULP_BLINK_RUN(delay_us, rtc_bit_); 
}

void ULPBlink::loop() {
    // Nothing here — ULP runs independently
}

void ULP_BLINK_RUN(uint32_t us, uint32_t bit) {

    RTC_SLOW_MEM[12] = 0;
    const ulp_insn_t  ulp_blink[] = {
        I_MOVI(R3, 12),                         // #12 -> R3
        I_LD(R0, R3, 0),                        // R0 = RTC_SLOW_MEM[R3(#12)] 
        M_BL(1, 1),                             // GOTO M_LABEL(1) IF R0 < 1
        I_WR_REG(RTC_GPIO_OUT_REG, bit, bit, 1),  // set pin high
        I_SUBI(R0, R0, 1),                      // R0 = R0 - 1, R0 = 1, R0 = 0
        I_ST(R0, R3, 0),                        // RTC_SLOW_MEM[R3(#12)] = R0
        M_BX(2),                                // GOTO M_LABEL(2)
        M_LABEL(1),                             // M_LABEL(1)
            I_WR_REG(RTC_GPIO_OUT_REG, bit, bit, 0),  // set pin low
            I_ADDI(R0, R0, 1),                    // R0 = R0 + 1, R0 = 0, R0 = 1
            I_ST(R0, R3, 0),                      // RTC_SLOW_MEM[R3(#12)] = R0
        M_LABEL(2),                             // M_LABEL(2)
        I_HALT()                                // HALT COPROCESSOR
    };

    // Microseconds to delay between halt and wake states
    ulp_set_wakeup_period(0, us);

    // Load and start ULP program
    size_t size = sizeof(ulp_blink) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_blink, &size);
    ulp_run(0);
}

void ULPBlink::dump_config() {
    if(rtc_bit_ < 0) {
        ESP_LOGE(TAG, "Pin %d is not RTC-capable", pin_->get_pin());
        return;
    }
    ESP_LOGCONFIG(TAG, "ULPBlink:");
    ESP_LOGCONFIG(TAG, "  Interval: %u ms", interval_);
    LOG_PIN("  Pin: ", pin_);
    ESP_LOGCONFIG(TAG, "  RTC Bit is %d", rtc_bit_);

}

}  // namespace ulp_blink
}  // namespace esphome