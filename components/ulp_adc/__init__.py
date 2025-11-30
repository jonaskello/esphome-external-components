import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN, CONF_INTERVAL
import esphome.pins as pins
import esphome.components.sensor as sensor

CONF_THRESHOLD = "threshold"

ulp_adc_ns = cg.esphome_ns.namespace("ulp_adc")
UlpAdc = ulp_adc_ns.class_("UlpAdc", cg.Component, sensor.Sensor)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UlpAdc),
        cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_THRESHOLD, default=100): cv.int_range(min=0, max=4095),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_interval(config[CONF_INTERVAL]))
    cg.add(var.set_threshold(config[CONF_THRESHOLD]))

