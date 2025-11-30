import logging

import esphome.codegen as cg
from esphome.components import sensor, voltage_sampler
from esphome.components.esp32 import get_esp32_variant
from esphome.config_helpers import filter_source_files_from_platform
import esphome.config_validation as cv
from esphome.const import (
    CONF_ATTENUATION,
    CONF_ID,
    CONF_NUMBER,
    CONF_PIN,
    CONF_RAW,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    PlatformFramework,
)
from esphome.core import CORE

from . import (
    ATTENUATION_MODES,
    ESP32_VARIANT_ADC1_PIN_TO_CHANNEL,
    adc_ulp_ns,
    adc_unit_t,
    validate_adc_pin,
)

_LOGGER = logging.getLogger(__name__)

AUTO_LOAD = ["voltage_sampler"]

_attenuation = cv.enum(ATTENUATION_MODES, lower=True)

def validate_config(config):
    if config[CONF_RAW] and config.get(CONF_ATTENUATION, None) == "auto":
        raise cv.Invalid("Automatic attenuation cannot be used when raw output is set")

    if config.get(CONF_ATTENUATION) == "11db":
        _LOGGER.warning(
            "`attenuation: 11db` is deprecated, use `attenuation: 12db` instead"
        )
        # Alter value here so `config` command prints the recommended change
        config[CONF_ATTENUATION] = _attenuation("12db")

    return config


ADCULPSensor = adc_ulp_ns.class_(
    "ADCULPSensor", sensor.Sensor, cg.Component, voltage_sampler.VoltageSampler
)

CONF_UPDATE_INTERVAL = "update_interval"

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        ADCULPSensor,
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Required(CONF_PIN): validate_adc_pin,
            cv.Optional(CONF_RAW, default=False): cv.boolean,
            cv.SplitDefault(CONF_ATTENUATION, esp32="0db"): cv.All(
                cv.only_on_esp32, _attenuation
            ),
            cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
        }
    ),
    validate_config,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_output_raw(config[CONF_RAW]))

    if update_interval := config.get(CONF_UPDATE_INTERVAL):
        cg.add(var.set_update_interval(update_interval))

    if attenuation := config.get(CONF_ATTENUATION):
        cg.add(var.set_attenuation(attenuation))

    variant = get_esp32_variant()
    pin_num = config[CONF_PIN][CONF_NUMBER]
    if (
        variant in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL
        and pin_num in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]
    ):
        chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
        cg.add(var.set_channel(chan))
    else:
        raise cv.Invalid(f"Pin {pin_num} is not supported by ULP ADC (ADC1 only)")

FILTER_SOURCE_FILES = filter_source_files_from_platform(
    {
        "adc_sensor_esp32.cpp": {
            PlatformFramework.ESP32_ARDUINO,
            PlatformFramework.ESP32_IDF,
        },
    }
)
