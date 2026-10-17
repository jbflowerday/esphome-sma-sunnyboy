"""SMA Sunny Boy inverter over Bluetooth Classic (SMA BT / SMANET2+)."""

import esphome.codegen as cg
from esphome.components import esp32
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MAC_ADDRESS, CONF_PASSWORD
from esphome.core import CORE

CODEOWNERS = []
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]
CONFLICTS_WITH = ["esp32_ble", "esp32_ble_tracker", "bluetooth_proxy"]

CONF_SMA_BLUETOOTH_ID = "sma_bluetooth_id"
CONF_USER_GROUP = "user_group"
CONF_RECONNECT_INTERVAL = "reconnect_interval"
CONF_ZERO_WHEN_OFFLINE = "zero_when_offline"

sma_bluetooth_ns = cg.esphome_ns.namespace("sma_bluetooth")
SmaBluetooth = sma_bluetooth_ns.class_("SmaBluetooth", cg.PollingComponent)

USER_GROUPS = {
    "user": 0x07,
    "installer": 0x0A,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SmaBluetooth),
            cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
            cv.Optional(CONF_PASSWORD, default="0000"): cv.All(
                cv.string, cv.Length(max=12)
            ),
            cv.Optional(CONF_USER_GROUP, default="user"): cv.enum(
                USER_GROUPS, lower=True
            ),
            cv.Optional(
                CONF_RECONNECT_INTERVAL, default="60s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ZERO_WHEN_OFFLINE, default=False): cv.boolean,
        }
    ).extend(cv.polling_component_schema("30s")),
    # Bluetooth Classic (BR/EDR) only exists on the original ESP32.
    esp32.only_on_variant(supported=[esp32.const.VARIANT_ESP32]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_address(config[CONF_MAC_ADDRESS].as_hex))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    cg.add(var.set_user_group(config[CONF_USER_GROUP]))
    cg.add(var.set_reconnect_interval(config[CONF_RECONNECT_INTERVAL]))
    cg.add(var.set_zero_when_offline(config[CONF_ZERO_WHEN_OFFLINE]))

    # Newer ESPHome excludes the "bt" IDF component unless a component asks for it.
    if hasattr(esp32, "request_bluetooth"):
        esp32.request_bluetooth()
        # Classic BT and WiFi share the radio; the coexistence arbiter is required.
        if "wifi" in CORE.config:
            esp32.request_software_coexistence()
    else:
        esp32.add_idf_sdkconfig_option("CONFIG_SW_COEXIST_ENABLE", True)

    # Bluedroid with Classic BT + SPP; BLE is disabled to save RAM and flash.
    esp32.add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_CLASSIC_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_SPP_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_BLE_ENABLED", False)
    esp32.add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BLE_ONLY", False)
    esp32.add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BTDM", False)
    esp32.add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY", True)
