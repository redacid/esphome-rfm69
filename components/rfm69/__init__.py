import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import pins
from esphome.const import CONF_ID, CONF_FREQUENCY

DEPENDENCIES = ["spi"]

rfm69_ns = cg.esphome_ns.namespace("rfm69")
RFM69Component = rfm69_ns.class_("RFM69Component", cg.Component)

CONF_CS_PIN = "cs_pin"
CONF_IRQ_PIN = "irq_pin"
CONF_RST_PIN = "rst_pin"
CONF_NETWORK_ID = "network_id"
CONF_NODE_ID = "node_id"
CONF_HIGH_POWER = "high_power"
CONF_POWER_LEVEL = "power_level"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(RFM69Component),
    cv.Required(CONF_CS_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
    cv.Optional(CONF_RST_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_FREQUENCY, default=868.0): cv.float_,
    cv.Optional(CONF_NETWORK_ID, default=100): cv.int_range(min=0, max=255),
    cv.Optional(CONF_NODE_ID, default=1): cv.int_range(min=0, max=255),
    cv.Optional(CONF_HIGH_POWER, default=True): cv.boolean,
    cv.Optional(CONF_POWER_LEVEL, default=20): cv.int_range(min=0, max=31),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cs = await cg.gpio_pin_expression(config[CONF_CS_PIN])
    cg.add(var.set_cs_pin(cs))

    irq = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
    cg.add(var.set_irq_pin(irq))

    if CONF_RST_PIN in config:
        rst = await cg.gpio_pin_expression(config[CONF_RST_PIN])
        cg.add(var.set_rst_pin(rst))

    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_network_id(config[CONF_NETWORK_ID]))
    cg.add(var.set_node_id(config[CONF_NODE_ID]))
    cg.add(var.set_high_power(config[CONF_HIGH_POWER]))
    cg.add(var.set_power_level(config[CONF_POWER_LEVEL]))