# ESPHome SMA Sunny Boy (Bluetooth)

ESPHome external component that reads an **SMA Sunny Boy SB 5000TL-21** (and other
SMA inverters with the classic SMA Bluetooth interface) over Bluetooth, then sends
the data to Home Assistant through the native ESPHome API.

You don't need SMA's cloud, Sunny Explorer, or a Raspberry Pi running SBFspot. An
ESP32 within Bluetooth range of the inverter does the job.

## Hardware

* An **original ESP32** (ESP32-WROOM / ESP32-WROVER / `esp32dev` boards).
  ESP32-S2, S3, C3, C6 and H2 **do not** support Bluetooth Classic and won't work.
* The inverter's Bluetooth must be on. The rotary switch "NetID" on the
  communication board must not be 0. The default is 1.
* Keep the ESP32 within roughly 10 m of the inverter with few walls in between.

## Installation

1. Copy `secrets.yaml.example` to `secrets.yaml` and `sma-sunnyboy.yaml.example`
   to `sma-sunnyboy.yaml` (both are git-ignored), then fill them in.
2. Set `inverter_mac` in `sma-sunnyboy.yaml` to the inverter's Bluetooth address.
   SMA addresses start with `00:80:25`. You can find it with Sunny Explorer, with
   `bluetoothctl scan on` / `hcitool scan` on Linux, or with any Bluetooth
   scanner app on a phone.
3. Build and flash:

   ```bash
   esphome run sma-sunnyboy.yaml
   ```

   On Windows, run this from PowerShell or cmd. The ESP-IDF installer refuses
   to run under Git Bash/MSYS.

4. Home Assistant discovers the device automatically
   (Settings → Devices & Services → ESPHome).

To use it from another ESPHome project, push this repo to GitHub and reference it:

```yaml
external_components:
  - source: github://<you>/esphome-sma-sunnyboy
    components: [sma_bluetooth]
```

## Configuration

```yaml
sma_bluetooth:
  mac_address: "00:80:25:xx:xx:xx"  # required
  password: "0000"                  # inverter User (or Installer) password, max 12 chars
  user_group: user                  # user | installer
  update_interval: 30s
  reconnect_interval: 60s
  zero_when_offline: false          # true: report 0 W at night instead of "unavailable"
```

| Platform | Keys |
|---|---|
| `sensor` | `ac_power`, `ac_voltage`, `ac_current`, `grid_frequency`, `dc_power` (A+B), `dc_power_a/b`, `dc_voltage_a/b`, `dc_current_a/b`, `energy_today`, `energy_total` (kWh, ready for the Energy dashboard), `operating_time`, `feed_in_time`, `temperature`, `bluetooth_signal` |
| `text_sensor` | `device_status` (Ok / Warning / Fault / Off / Offline), `grid_relay`, `device_name`, `serial_number`, `software_version` |
| `binary_sensor` | `connected` |

Every key is optional. Add only the ones you want. `sma-sunnyboy.yaml.example` shows them all.

To track solar production in the Home Assistant **Energy dashboard**, use the
`energy_total` sensor.

## How it works

The component uses the ESP-IDF Bluedroid SPP API directly. It doesn't depend on
the Arduino `BluetoothSerial` library. It opens an RFCOMM connection on channel
1 to the inverter and speaks the SMA "SMANET2+" protocol, which was
reverse-engineered by the [SBFspot](https://github.com/SBFspot/SBFspot) project:

1. L1 handshake: version request, `0x0002` announcement and reply, then `0x000A` / `0x0005` network info.
2. SMANET2 identification broadcast (reads the inverter's SUSyID and serial number).
3. Logon with the User/Installer password.
4. Spot-value queries on every `update_interval`. The connection stays open between polls.

At dusk the inverter turns its Bluetooth off. The component notices that the
connection dropped and marks the live values unavailable, or 0 if you set
`zero_when_offline`. It then retries every `reconnect_interval` until the
inverter wakes up again.

## Limitations / notes

* Bluetooth Classic and BLE can't be used together here. Don't combine this with
  `esp32_ble_tracker` or `bluetooth_proxy` on the same ESP32.
* Wi-Fi and Bluetooth share one radio. This works fine, but a weak Wi-Fi signal
  combined with a weak Bluetooth signal can cause dropouts.
* Only one Bluetooth client can talk to the inverter at a time. Close Sunny
  Explorer and stop any SBFspot instances.
* The component talks to a single inverter. Multi-inverter Bluetooth networks
  (NetID > 1 with several devices) aren't supported.
* If the log shows `Logon rejected: wrong password`, check the password and the
  user group. The default User password is `0000`.

## Debugging

Set `logger: level: VERBOSE` and add `logs: sma_bluetooth: VERBOSE` to see every
L1 frame and decoded record.
