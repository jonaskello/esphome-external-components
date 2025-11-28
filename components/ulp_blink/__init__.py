import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN
import esphome.pins as pins

ulp_blink_ns = cg.esphome_ns.namespace("ulp_blink")
ULPBlink = ulp_blink_ns.class_("ULPBlink", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ULPBlink),
        cv.Required(CONF_PIN): pins.gpio_output_pin_schema,
        cv.Optional("interval", default="1s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_interval(config["interval"]))
