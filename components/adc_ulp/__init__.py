from esphome import pins
import esphome.codegen as cg
from esphome.components.esp32 import VARIANT_ESP32P4, get_esp32_variant
from esphome.components.esp32.const import (
    VARIANT_ESP32,
    VARIANT_ESP32S2,
    VARIANT_ESP32S3,
)
import esphome.config_validation as cv
from esphome.const import CONF_NUMBER
from esphome.core import CORE

CODEOWNERS = ["jonaskello"]

adc_ns = cg.esphome_ns.namespace("adc")

"""
From the below patch versions (and 5.2+) ADC_ATTEN_DB_11 is deprecated and replaced with ADC_ATTEN_DB_12.
4.4.7
5.0.5
5.1.3
5.2+
"""

ATTENUATION_MODES = {
    "0db": cg.global_ns.ADC_ATTEN_DB_0,
    "2.5db": cg.global_ns.ADC_ATTEN_DB_2_5,
    "6db": cg.global_ns.ADC_ATTEN_DB_6,
    "11db": adc_ns.ADC_ATTEN_DB_12_COMPAT,
    "12db": adc_ns.ADC_ATTEN_DB_12_COMPAT,
    "auto": "auto",
}

sampling_mode = adc_ns.enum("SamplingMode", is_class=True)

SAMPLING_MODES = {
    "avg": sampling_mode.AVG,
    "min": sampling_mode.MIN,
    "max": sampling_mode.MAX,
}

adc_unit_t = cg.global_ns.enum("adc_unit_t", is_class=True)

adc_channel_t = cg.global_ns.enum("adc_channel_t", is_class=True)

# pin to adc1 channel mapping
# https://github.com/espressif/esp-idf/blob/v4.4.8/components/driver/include/driver/adc.h
ESP32_VARIANT_ADC1_PIN_TO_CHANNEL = {
    # https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/adc_channel.h
    VARIANT_ESP32: {
        36: adc_channel_t.ADC_CHANNEL_0,
        37: adc_channel_t.ADC_CHANNEL_1,
        38: adc_channel_t.ADC_CHANNEL_2,
        39: adc_channel_t.ADC_CHANNEL_3,
        32: adc_channel_t.ADC_CHANNEL_4,
        33: adc_channel_t.ADC_CHANNEL_5,
        34: adc_channel_t.ADC_CHANNEL_6,
        35: adc_channel_t.ADC_CHANNEL_7,
    },
    # https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s2/include/soc/adc_channel.h
    VARIANT_ESP32S2: {
        1: adc_channel_t.ADC_CHANNEL_0,
        2: adc_channel_t.ADC_CHANNEL_1,
        3: adc_channel_t.ADC_CHANNEL_2,
        4: adc_channel_t.ADC_CHANNEL_3,
        5: adc_channel_t.ADC_CHANNEL_4,
        6: adc_channel_t.ADC_CHANNEL_5,
        7: adc_channel_t.ADC_CHANNEL_6,
        8: adc_channel_t.ADC_CHANNEL_7,
        9: adc_channel_t.ADC_CHANNEL_8,
        10: adc_channel_t.ADC_CHANNEL_9,
    },
    # https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s3/include/soc/adc_channel.h
    VARIANT_ESP32S3: {
        1: adc_channel_t.ADC_CHANNEL_0,
        2: adc_channel_t.ADC_CHANNEL_1,
        3: adc_channel_t.ADC_CHANNEL_2,
        4: adc_channel_t.ADC_CHANNEL_3,
        5: adc_channel_t.ADC_CHANNEL_4,
        6: adc_channel_t.ADC_CHANNEL_5,
        7: adc_channel_t.ADC_CHANNEL_6,
        8: adc_channel_t.ADC_CHANNEL_7,
        9: adc_channel_t.ADC_CHANNEL_8,
        10: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32P4: {
        16: adc_channel_t.ADC_CHANNEL_0,
        17: adc_channel_t.ADC_CHANNEL_1,
        18: adc_channel_t.ADC_CHANNEL_2,
        19: adc_channel_t.ADC_CHANNEL_3,
        20: adc_channel_t.ADC_CHANNEL_4,
        21: adc_channel_t.ADC_CHANNEL_5,
        22: adc_channel_t.ADC_CHANNEL_6,
        23: adc_channel_t.ADC_CHANNEL_7,
    },
}

def validate_adc_pin(value):

    if not CORE.is_esp32:
        raise cv.Invalid("ULP ADC is only supported on ESP32 boards")

    conf = pins.internal_gpio_input_pin_schema(value)
    value = conf[CONF_NUMBER]
    variant = get_esp32_variant()

    if variant not in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL:
        raise cv.Invalid(f"This ESP32 variant ({variant}) is not supported")

    if value not in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]:
        raise cv.Invalid(f"{variant} doesn't support ADC on this pin")

    return conf
