#include "esphome/core/log.h"
#include "simple_blink.h"

namespace esphome {
namespace simple_blink {

static const char *TAG = "simple_blink.component";

void SimpleBlink::setup() {
    if (pin_ != nullptr) {
        pin_->setup();
        pin_->pin_mode(gpio::FLAG_OUTPUT);
        pin_->digital_write(false);  // start OFF
    }
}

void SimpleBlink::loop() {
    if (pin_ == nullptr) return;

    uint32_t now = millis();
    if (now - last_toggle_ >= interval_) {
        state_ = !state_;
        pin_->digital_write(state_);
        last_toggle_ = now;
    }
}

void SimpleBlink::dump_config() {
    ESP_LOGCONFIG(TAG, "SimpleBlink:");
    ESP_LOGCONFIG(TAG, "  Interval: %u ms", interval_);
    LOG_PIN("  Pin: ", pin_);
}

}  // namespace simple_blink
}  // namespace esphome
