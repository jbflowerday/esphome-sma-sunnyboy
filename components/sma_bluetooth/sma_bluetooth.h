#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <atomic>
#include <deque>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include <esp_bt_defs.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>

namespace esphome {
namespace sma_bluetooth {

enum class State : uint8_t {
  BT_INIT,         // waiting for the Bluedroid SPP stack to come up
  IDLE,            // not connected, waiting for the next reconnect attempt
  CONNECTING,      // RFCOMM connect in progress
  WAIT_HELLO,      // waiting for the inverter's 0x0002 announcement
  WAIT_NET_READY,  // waiting for 0x000A / 0x0005 network topology packets
  WAIT_DISCOVERY,  // waiting for the SMANET2 identification reply
  WAIT_LOGON,      // waiting for the logon reply
  READY,           // logged in, idle between polls
  WAIT_REPLY,      // waiting for the reply to a data query
};

struct Query {
  uint32_t command;
  uint32_t first;
  uint32_t last;
};

class SmaBluetooth : public PollingComponent {
 public:
  SUB_SENSOR(ac_power)
  SUB_SENSOR(ac_voltage)
  SUB_SENSOR(ac_current)
  SUB_SENSOR(grid_frequency)
  SUB_SENSOR(dc_power)
  SUB_SENSOR(dc_power_a)
  SUB_SENSOR(dc_voltage_a)
  SUB_SENSOR(dc_current_a)
  SUB_SENSOR(dc_power_b)
  SUB_SENSOR(dc_voltage_b)
  SUB_SENSOR(dc_current_b)
  SUB_SENSOR(energy_today)
  SUB_SENSOR(energy_total)
  SUB_SENSOR(operating_time)
  SUB_SENSOR(feed_in_time)
  SUB_SENSOR(temperature)
  SUB_SENSOR(bluetooth_signal)
  SUB_TEXT_SENSOR(device_status)
  SUB_TEXT_SENSOR(grid_relay)
  SUB_TEXT_SENSOR(device_name)
  SUB_TEXT_SENSOR(serial_number)
  SUB_TEXT_SENSOR(software_version)
  SUB_BINARY_SENSOR(connected)

 public:
  void set_address(uint64_t address);
  void set_password(const std::string &password) { this->password_ = password; }
  void set_user_group(uint32_t user_group) { this->user_group_ = user_group; }
  void set_reconnect_interval(uint32_t interval) { this->reconnect_interval_ = interval; }
  void set_zero_when_offline(bool zero) { this->zero_when_offline_ = zero; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

 protected:
  static void spp_callback_(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
  static void gap_callback_(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
  static SmaBluetooth *instance_;  // NOLINT

  bool init_bluetooth_();
  void handle_bt_events_();
  void set_state_(State state);
  void start_connect_();
  void disconnect_(const char *reason);
  void on_disconnected_();

  // Transport
  void process_rx_();
  void handle_l1_frame_(const uint8_t *frame, size_t len);
  void handle_l1_(uint16_t cmd, const uint8_t *payload, size_t len);
  void handle_l2_(const std::vector<uint8_t> &pkt);
  void flush_tx_();
  void send_l1_(uint16_t cmd, const uint8_t *dst, const std::vector<uint8_t> &payload);
  uint16_t send_l2_(uint8_t ctrl, uint16_t ctrl2, uint16_t dst_susy, uint32_t dst_serial,
                    const std::vector<uint8_t> &data);

  // Protocol steps
  void send_hello_reply_();
  void send_discovery_();
  void send_logoff_();
  void send_logon_();
  void send_signal_request_();
  void start_poll_cycle_();
  void send_next_query_();
  void parse_records_(const std::vector<uint8_t> &pkt);
  void publish_offline_();

  // Configuration
  esp_bd_addr_t peer_bda_{};      // inverter address, MSB first (ESP-IDF order)
  uint8_t inverter_addr_[6]{};    // inverter address, LSB first (SMA packet order)
  uint8_t local_addr_[6]{};       // our address, LSB first (SMA packet order)
  std::string password_;
  uint32_t user_group_{0x07};
  uint32_t reconnect_interval_{60000};
  bool zero_when_offline_{false};

  // Bluetooth state shared with the Bluedroid task
  StreamBufferHandle_t rx_stream_{nullptr};
  std::atomic<bool> spp_ready_{false};
  std::atomic<bool> ev_open_{false};
  std::atomic<bool> ev_open_failed_{false};
  std::atomic<bool> ev_closed_{false};
  std::atomic<bool> tx_busy_{false};
  std::atomic<bool> tx_congested_{false};
  std::atomic<uint32_t> spp_handle_{0};
  std::atomic<int> last_status_{0};

  // Main loop state
  State state_{State::BT_INIT};
  uint32_t state_since_{0};
  uint32_t next_attempt_{0};
  uint32_t tx_since_{0};
  bool got_net_info_{false};
  uint8_t net_id_{1};
  std::vector<uint8_t> rx_buf_;
  std::vector<uint8_t> l2_buf_;
  bool l2_escape_{false};
  std::deque<std::vector<uint8_t>> tx_queue_;

  // SMANET2 session
  uint32_t app_serial_{0};
  uint16_t packet_id_{1};
  uint16_t expected_packet_id_{0};
  uint32_t logon_time_{0};
  uint32_t logon_millis_{0};
  uint16_t inverter_susy_{0xFFFF};
  uint32_t inverter_serial_{0xFFFFFFFF};

  // Poll cycle
  std::vector<Query> queries_;
  size_t query_index_{0};
  bool info_fetched_{false};
  bool poll_requested_{false};
  uint8_t consecutive_timeouts_{0};
  float dc_power_[2]{NAN, NAN};
};

}  // namespace sma_bluetooth
}  // namespace esphome
