#ifdef USE_ESP32

#include "adc_ulp.h"
#include "esphome/core/log.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "esphome.h"
#include "esphome/core/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_sleep.h"
#include "esphome/core/util.h"

namespace esphome {
namespace adc_ulp {

static const char *const TAG = "adc_ulp.esp32";

// 5 seconds for deep sleep to ensure clean disconnect from Home Assistant
static const uint32_t TEARDOWN_TIMEOUT_DEEP_SLEEP_MS = 5000;

#define DATA_BASE_SLOT     64
#define BASELINE_OFFSET    0
#define ARM_OFFSET         1
#define THRESHOLD_UP_OFFSET   2
#define THRESHOLD_DOWN_OFFSET 3
#define DEBUG1_OFFSET      4
#define DEBUG2_OFFSET      5
#define DEBUG3_OFFSET      6
#define DEBUG4_OFFSET      7

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
            this->mark_failed("Error initializing ADC1");
            return;
        }
        this->setup_flags_.handle_init_complete = true;

        // Init ADC channel for use with ULP
        adc_oneshot_chan_cfg_t chan_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
        esp_err_t err_channel = adc_oneshot_config_channel(adc1_handle, channel_, &chan_cfg);
        if (err_channel != ESP_OK) {
            ESP_LOGE(TAG, "Error configuring ADC channel: %d", err_channel);
            this->mark_failed("Error configuring ADC channel");
            return;
        }
        this->setup_flags_.config_complete = true;

        // Set baseline out of range baseline to trigger first value publish
        RTC_SLOW_MEM[DATA_BASE_SLOT + BASELINE_OFFSET] = 0x7FFF;
        // Prevent measurements when CPU is awake by setting ARM = 0
        RTC_SLOW_MEM[DATA_BASE_SLOT + ARM_OFFSET] = 0;
        RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG1_OFFSET] = 1;
        RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG2_OFFSET] = 2;
        RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG3_OFFSET] = 3;
        RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG4_OFFSET] = 4;

        // Define ULP program
        const ulp_insn_t ulp_prog[] = {

            // Use R3 as data pointer in the whole program
            I_MOVI(R3, DATA_BASE_SLOT), // R3 = base data pointer

            // Check ARM flag so we don't measure values when the CPU is awake
            I_LD(R0, R3, ARM_OFFSET),  // R0 = ARM
            M_BL(2, 1),                // if ARM < 1 it is 0 (it can be 1 or 0) → branch to label 2 (skip)

            // I_ST(R1, R3, DEBUG1_OFFSET),  // DEBUG
            // I_ST(R2, R3, DEBUG2_OFFSET),  // DEBUG

            // Run real program when armed
            // Read the ADC
            I_ADC(R2, 0, (uint32_t)channel_),  // R2 = measured raw ADC
            // Check upward threshold
            I_LD(R1, R3, BASELINE_OFFSET),     // R1 = baseline
            I_SUBR(R0, R2, R1),                // R0 = measured - baseline (delta relative to baseline)
            I_LD(R1, R3, THRESHOLD_UP_OFFSET), // R1 = upward threshold
            I_SUBR(R0, R0, R1),                // R0 = delta - threshold (margin relative to threshold)
            I_RSHI(R0, R0, 15),                // R0 = sign bit of R0 (0 if ≥0, 1 if negative)
            M_BL(1, 1),                        // if signbit < 1 (i.e 0) then delta > threshold so wakeup
            // Check downward threshold
            I_LD(R1, R3, BASELINE_OFFSET),     // R1 = baseline
            I_SUBR(R0, R1, R2),                // R0 = baseline - measured (absolute delta relative to baseline)
            I_LD(R1, R3, THRESHOLD_DOWN_OFFSET), // R1 = downward threshold
            I_SUBR(R0, R0, R1),                // R0 = delta - threshold (margin relative to threshold)
            I_RSHI(R0, R0, 15),                // R0 = sign bit of R0 (0 if ≥0, 1 if negative)
            M_BL(1, 1),                        // if signbit < 1 (i.e 0) then delta > threshold so wakeup
            // Skip to end and do not wake
            M_BX(2),                           // else skip wake
            // Wakeup label
            M_LABEL(1),
                I_ST(R2, R3, BASELINE_OFFSET),  // update baseline with raw ADC
                I_MOVI(R0, 0),                  // R0 = 0 to clear ARM
                I_ST(R0, R3, ARM_OFFSET),       // clear ARM before we wake the cpu so we don't run while it is awake
                I_WAKE(),                       // wake CPU
            M_LABEL(2),
                I_HALT()                        // halt
        };

        // Load ULP program into RTC memory
        size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
        if(size > DATA_BASE_SLOT) {
            ESP_LOGE(TAG, "ULP program larger than DATA_BASE_SLOT");
            this->mark_failed("ULP program larger than DATA_BASE_SLOT");
            return;
        }
        esp_err_t r = ulp_process_macros_and_load(0, ulp_prog, &size);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "ulp_process_macros_and_load failed: %d", r);
            this->mark_failed("ulp_process_macros_and_load failed");
            return;
        }
        ESP_LOGI(TAG, "First power on, init ULP completed, program size: %d", size);
        // dump_config();
    }
    else {
        // This is already setup
        this->setup_flags_.handle_init_complete = true;
        this->setup_flags_.config_complete = true;

        // Calibration handle only used in this branch
        setup_calibration_();

        // Publish only on wakeup from ULP
        uint32_t raw_measure = RTC_SLOW_MEM[DATA_BASE_SLOT + BASELINE_OFFSET];
        uint16_t actual_measure = raw_measure & 0x0FFF; // Only 12 bits are used
        auto converted_value = convert_fixed_attenuation_(actual_measure);
        publish_state(converted_value);
        ESP_LOGI(TAG, "Published ADC value: %f", converted_value);
    }

    init_raw_thresholds();
    this->setup_flags_.init_complete = true;

    // Debug print
    ESP_LOGI(TAG, "BASELINE_OFFSET: %u", RTC_SLOW_MEM[DATA_BASE_SLOT + BASELINE_OFFSET] & 0xFFFF);
    ESP_LOGI(TAG, "THRESHOLD_UP_OFFSET: %u", RTC_SLOW_MEM[DATA_BASE_SLOT + THRESHOLD_UP_OFFSET] & 0xFFFF);
    ESP_LOGI(TAG, "THRESHOLD_DOWN_OFFSET: %u", RTC_SLOW_MEM[DATA_BASE_SLOT + THRESHOLD_DOWN_OFFSET] & 0xFFFF);

    // ESP_LOGI(TAG, "DEBUG1_OFFSET: %d", (int16_t)(RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG1_OFFSET] & 0xFFFF));
    // ESP_LOGI(TAG, "DEBUG2_OFFSET: %d", (int16_t)(RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG2_OFFSET] & 0xFFFF));
    // ESP_LOGI(TAG, "DEBUG3_OFFSET: %d", (int16_t)(RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG3_OFFSET] & 0xFFFF));
    // ESP_LOGI(TAG, "DEBUG4_OFFSET: %d", (int16_t)(RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG4_OFFSET] & 0xFFFF));
    // uint32_t debug3 = RTC_SLOW_MEM[DATA_BASE_SLOT + DEBUG3_OFFSET];
    // ESP_LOGI(TAG, "DEBUG3 raw: 0x%08X (PC=%u, reg=%u, val=%d)", debug3, (debug3 >> 21) & 0x7FF, (debug3 >> 16) & 0x3, (int16_t)(debug3 & 0xFFFF));

}

float ADCULPSensor::get_setup_priority() const { return setup_priority::LATE; }

float ADCULPSensor::get_loop_priority() const {
  return -100.0f;  // run after everything else is ready
}

void ADCULPSensor::loop() {

    // Wait for remote connection so the published value is sent
    static unsigned long t0 = 0, last = 0;
    static bool gave_up = false;
    const int wait_ms = 80000, log_ms = 1000;
    if(t0 == 0) {
        ESP_LOGI(TAG, "Wait %d ms for remote to connect...", wait_ms);
    }
    bool remote_connected = remote_is_connected();
    // bool remote_connected = api::global_api_server->is_connected();
    if (!remote_connected && !gave_up) {
        if (!t0) t0 = millis();
        if (millis() - last >= log_ms) { last = millis(); ESP_LOGI(TAG, "Waiting for remote connection (api or mqtt)... %d", (last - t0) / 1000); }
        if (millis() - t0 > wait_ms) { gave_up = true; }
        return;
    }
    if(gave_up) {
        ESP_LOGI(TAG, "Timeout while waiting for remote, published values will not be sent");
    }
    else {
        ESP_LOGI(TAG, "Remote connected...");
        // const int send_delay = 500;
        // delay(send_delay);
    }
    t0 = 0;  // connected: reset timeout state
    gave_up = false;

    // Tell esphome this was "successful" to not trigger safe mode
    App.run_safe_shutdown_hooks();
    // It's critical to teardown components cleanly for deep sleep to ensure
    // Home Assistant sees a clean disconnect instead of marking the device unavailable
    App.teardown_components(TEARDOWN_TIMEOUT_DEEP_SLEEP_MS);
    App.run_powerdown_hooks();

    // Microseconds to delay between ULP halt and wake states
    esp_err_t r = ulp_set_wakeup_period(0, update_interval_ms_ * 1000);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ulp_set_wakeup_period failed: %d", r);
        this->mark_failed();
        return;
    }
    // Start ULP program
    r = ulp_run(0);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ulp_run failed: %d", r);
        this->mark_failed();
        return;
    }

    // Enable wakeup from ULP
    esp_sleep_enable_ulp_wakeup();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Last log before sleep
    ESP_LOGI(TAG, "Entering deep sleep until next ULP wake...");

    // Wait for log
    // fflush(stdout);
    // uart_wait_tx_idle_polling(static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM));

    // Tell ULP to start making measurements
    RTC_SLOW_MEM[DATA_BASE_SLOT + ARM_OFFSET] = 1;
    // Enter sleep
    esp_deep_sleep_start();

}

void ADCULPSensor::dump_config() {
    LOG_SENSOR("", "ADC ULP Sensor", this);
    LOG_PIN("  Pin: ", this->pin_);
    ESP_LOGCONFIG(TAG,
                "  Channel:       %d\n"
                "  Unit:          ADC1\n"
                "  Threshold:     %.2f\n"
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

void ADCULPSensor::setup_calibration_() {

    // Initialize ADC calibration
    if (this->calibration_handle_ == nullptr) {
        adc_cali_handle_t handle = nullptr;

    #if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C5 || USE_ESP32_VARIANT_ESP32C6 || \
        USE_ESP32_VARIANT_ESP32S3 || USE_ESP32_VARIANT_ESP32H2 || USE_ESP32_VARIANT_ESP32P4
        // RISC-V variants and S3 use curve fitting calibration
        adc_cali_curve_fitting_config_t cali_config = {};  // Zero initialize first
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
        cali_config.chan = this->channel_;
    #endif  // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
        // cali_config.unit_id = this->adc_unit_;
        cali_config.unit_id = ADC_UNIT_1;
        cali_config.atten = this->attenuation_;
        cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;

        esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (err == ESP_OK) {
            this->calibration_handle_ = handle;
            this->setup_flags_.calibration_complete = true;
            ESP_LOGV(TAG, "Using curve fitting calibration");
        } else {
            ESP_LOGW(TAG, "Curve fitting calibration failed with error %d, will use uncalibrated readings", err);
            this->setup_flags_.calibration_complete = false;
        }
    #else  // Other ESP32 variants use line fitting calibration
        adc_cali_line_fitting_config_t cali_config = {
        // .unit_id = this->adc_unit_,
        .unit_id = ADC_UNIT_1,
        .atten = this->attenuation_,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    #if !defined(USE_ESP32_VARIANT_ESP32S2)
        .default_vref = 1100,  // Default reference voltage in mV
    #endif  // !defined(USE_ESP32_VARIANT_ESP32S2)
        };
        esp_err_t err = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (err == ESP_OK) {
            this->calibration_handle_ = handle;
            this->setup_flags_.calibration_complete = true;
            ESP_LOGV(TAG, "Using line fitting calibration");
        } else {
            ESP_LOGW(TAG, "Line fitting calibration failed with error %d, will use uncalibrated readings", err);
            this->setup_flags_.calibration_complete = false;
        }
    #endif  // USE_ESP32_VARIANT_ESP32C3 || ESP32C5 || ESP32C6 || ESP32S3 || ESP32H2
    }

}

float ADCULPSensor::convert_fixed_attenuation_(uint32_t raw_value) {

    if (this->output_raw_) {
        return raw_value;
    }

    if (this->calibration_handle_ != nullptr) {
        int voltage_mv;
        esp_err_t err = adc_cali_raw_to_voltage(this->calibration_handle_, raw_value, &voltage_mv);
        if (err == ESP_OK) {
            return voltage_mv / 1000.0f;
        } else {
            ESP_LOGW(TAG, "ADC calibration conversion failed with error %d, disabling calibration", err);
            if (this->calibration_handle_ != nullptr) {
            #if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C5 || USE_ESP32_VARIANT_ESP32C6 || \
                USE_ESP32_VARIANT_ESP32S3 || USE_ESP32_VARIANT_ESP32H2 || USE_ESP32_VARIANT_ESP32P4
                adc_cali_delete_scheme_curve_fitting(this->calibration_handle_);
            #else   // Other ESP32 variants use line fitting calibration
                adc_cali_delete_scheme_line_fitting(this->calibration_handle_);
            #endif  // USE_ESP32_VARIANT_ESP32C3 || ESP32C5 || ESP32C6 || ESP32S3 || ESP32H2
                this->calibration_handle_ = nullptr;
            }
        }
    }

    return raw_value * 3.3f / 4095.0f;
}

void ADCULPSensor::init_raw_thresholds() {
    uint32_t threshold_up   = 0;
    uint32_t threshold_down = 0;

    if (this->output_raw_) {
        // Threshold is already in raw counts
        threshold_up   = this->threshold_;
        threshold_down = this->threshold_;
    } else {
        // Convert volts → raw counts relative to baseline (because attenuation curve is not linear)
        uint32_t baseline_raw = RTC_SLOW_MEM[DATA_BASE_SLOT + BASELINE_OFFSET] & 0xFFFF;
        if (baseline_raw > 4095) {
            // Initial bootstrap: force a wake on first measure
            threshold_up   = 10;
            threshold_down = 10;
        } else {
            // Convert current baseline to voltage
            float baseline_v = convert_fixed_attenuation_(baseline_raw);

            // Upward: target = baseline_v + threshold_V → raw_target_up
            float target_up_v = baseline_v + this->threshold_;
            uint16_t raw_target_up = voltage_to_raw(target_up_v);
            threshold_up = (raw_target_up > baseline_raw) ? (raw_target_up - baseline_raw) : 0;

            // Downward: target = baseline_v - threshold_V → raw_target_down
            float target_down_v = baseline_v - this->threshold_;
            if (target_down_v < 0.0f) target_down_v = 0.0f; // floor at 0 V
            uint16_t raw_target_down = voltage_to_raw(target_down_v);
            threshold_down = (baseline_raw > raw_target_down) ? (baseline_raw - raw_target_down) : 0;

            // Enforce minimum non-zero thresholds to "sleep on equality"
            if (threshold_up == 0 && this->threshold_ > 0) threshold_up = 1;
            if (threshold_down == 0 && this->threshold_ > 0) threshold_down = 1;
        }
    }

    // Finally transfer to RTC slow memory
    RTC_SLOW_MEM[DATA_BASE_SLOT + THRESHOLD_UP_OFFSET]   = threshold_up & 0xFFFF;
    RTC_SLOW_MEM[DATA_BASE_SLOT + THRESHOLD_DOWN_OFFSET] = threshold_down & 0xFFFF;
}

uint32_t ADCULPSensor::voltage_to_raw(float target_v) {
    if (this->calibration_handle_ != nullptr) {
        int lo = 0, hi = 4095, ans = -1;  // default to -1
        while (lo <= hi) {
            int mid = (lo + hi) / 2;
            int mv;
            esp_err_t err = adc_cali_raw_to_voltage(this->calibration_handle_, mid, &mv);
            if (err != ESP_OK) {
                // Calibration failed
                ans = -1;
                ESP_LOGW(TAG, "ADC calibration conversion failed with error %d, disabling calibration", err);
                if (this->calibration_handle_ != nullptr) {
                #if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C5 || USE_ESP32_VARIANT_ESP32C6 || \
                    USE_ESP32_VARIANT_ESP32S3 || USE_ESP32_VARIANT_ESP32H2 || USE_ESP32_VARIANT_ESP32P4
                    adc_cali_delete_scheme_curve_fitting(this->calibration_handle_);
                #else   // Other ESP32 variants use line fitting calibration
                    adc_cali_delete_scheme_line_fitting(this->calibration_handle_);
                #endif  // USE_ESP32_VARIANT_ESP32C3 || ESP32C5 || ESP32C6 || ESP32S3 || ESP32H2
                    this->calibration_handle_ = nullptr;
                }
                break;
            }
            float v = mv / 1000.0f;
            ans = mid;
            if (v < target_v) lo = mid + 1;
            else hi = mid - 1;
        }
        // If calibration success return, otherwise fall through to fallback
        if(ans != -1) {
            return ans; 
        }
    }

    // Fallback
    int raw = static_cast<uint32_t>(target_v / 3.3f * 4095.0f);
    return raw < 0 ? 0 : (raw > 4095 ? 4095 : raw);
}

}  // namespace adc_ulp
}  // namespace esphome

#endif  // USE_ESP32
