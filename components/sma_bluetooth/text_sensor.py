import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_SMA_BLUETOOTH_ID, SmaBluetooth

DEPENDENCIES = ["sma_bluetooth"]

TEXT_SENSORS = {
    "device_status": text_sensor.text_sensor_schema(icon="mdi:solar-power"),
    "grid_relay": text_sensor.text_sensor_schema(icon="mdi:electric-switch"),
    "device_name": text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:information-outline"
    ),
    "serial_number": text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:identifier"
    ),
    "software_version": text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:chip"
    ),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SMA_BLUETOOTH_ID): cv.use_id(SmaBluetooth),
        **{cv.Optional(key): schema for key, schema in TEXT_SENSORS.items()},
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SMA_BLUETOOTH_ID])
    for key in TEXT_SENSORS:
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
