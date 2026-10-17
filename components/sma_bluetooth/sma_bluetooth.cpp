// SMA Sunny Boy Bluetooth (SMANET2+ over RFCOMM) client for ESPHome.
//
// Protocol details follow SBFspot (https://github.com/SBFspot/SBFspot):
//   Level 1 frame: 7E len_lo len_hi chk | src[6] | dst[6] | cmd[2] | payload
//   Level 2 packet (cmd 0x0001 = last fragment, 0x0008 = more fragments follow):
//     7E | HDLC-escaped(FF 03 60 65 | header | data) | FCS16 | 7E
//   Addresses inside frames are sent LSB first.

#include "sma_bluetooth.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <cstring>
#include <ctime>

#include <esp_bt.h>
#include <esp_bt_main.h>
#if __has_include(<esp_mac.h>)
#include <esp_mac.h>
#else
#include <esp_system.h>
#endif

namespace esphome {
namespace sma_bluetooth {

static const char *const TAG = "sma_bluetooth";

SmaBluetooth *SmaBluetooth::instance_ = nullptr;  // NOLINT

static const uint8_t SPP_CHANNEL = 1;
static const uint16_t APP_SUSY_ID = 125;
static const uint32_t L2_SIGNATURE = 0x656003FF;
static const size_t L1_HEADER_SIZE = 18;
static const size_t L2_DATA_OFFSET = 41;

static const uint16_t CMD_L2_LAST = 0x0001;
static const uint16_t CMD_HELLO = 0x0002;
static const uint16_t CMD_SIGNAL_REQUEST = 0x0003;
static const uint16_t CMD_SIGNAL_REPLY = 0x0004;
static const uint16_t CMD_TOPOLOGY = 0x0005;
static const uint16_t CMD_L2_PART = 0x0008;
static const uint16_t CMD_NET_INFO = 0x000A;
static const uint16_t CMD_VERSION = 0x0201;

static const uint8_t ADDR_UNKNOWN[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t ADDR_VERSION[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t BT_PIN[4] = {'0', '0', '0', '0'};

static const uint32_t CONNECT_TIMEOUT = 30000;
static const uint32_t HELLO_TIMEOUT = 15000;
static const uint32_t NET_READY_TIMEOUT = 5000;
static const uint32_t REPLY_TIMEOUT = 5000;
static const uint32_t RELOGON_INTERVAL = 600000;  // session timeout requested at logon is 900 s
static const uint8_t MAX_CONSECUTIVE_TIMEOUTS = 3;

// LRI codes (record code & 0x00FFFF00)
static const uint32_t LRI_OPERATION_HEALTH = 0x00214800;
static const uint32_t LRI_TEMPERATURE = 0x00237700;
static const uint32_t LRI_DC_POWER = 0x00251E00;
static const uint32_t LRI_ENERGY_TOTAL = 0x00260100;
static const uint32_t LRI_ENERGY_TODAY = 0x00262200;
static const uint32_t LRI_AC_POWER_TOTAL = 0x00263F00;
static const uint32_t LRI_GRID_RELAY = 0x00416400;
static const uint32_t LRI_DC_VOLTAGE = 0x00451F00;
static const uint32_t LRI_DC_CURRENT = 0x00452100;
static const uint32_t LRI_OPERATING_TIME = 0x00462E00;
static const uint32_t LRI_FEED_IN_TIME = 0x00462F00;
static const uint32_t LRI_AC_VOLTAGE_A = 0x00464800;
static const uint32_t LRI_AC_CURRENT_A_1 = 0x00465000;
static const uint32_t LRI_AC_CURRENT_A = 0x00465300;
static const uint32_t LRI_GRID_FREQUENCY = 0x00465700;
static const uint32_t LRI_DEVICE_NAME = 0x00821E00;
static const uint32_t LRI_SOFTWARE_VERSION = 0x00823400;

static const uint8_t DT_STATUS = 0x08;
static const uint8_t DT_STRING = 0x10;

static const Query INFO_QUERIES[] = {
    {0x58000200, 0x00821E00, 0x008220FF},  // type label (device name, class, type)
    {0x58000200, 0x00823400, 0x008234FF},  // software version
};

static const Query SPOT_QUERIES[] = {
    {0x51000200, 0x00263F00, 0x00263FFF},  // AC total power
    {0x51000200, 0x00464800, 0x004655FF},  // AC voltage / current
    {0x51000200, 0x00465700, 0x004657FF},  // grid frequency
    {0x53800200, 0x00251E00, 0x00251EFF},  // DC power per MPP tracker
    {0x53800200, 0x00451F00, 0x004521FF},  // DC voltage / current per MPP tracker
    {0x54000200, 0x00260100, 0x002622FF},  // energy total / today
    {0x54000200, 0x00462E00, 0x00462FFF},  // operating / feed-in time
    {0x51800200, 0x00214800, 0x002148FF},  // device status
    {0x51800200, 0x00416400, 0x004164FF},  // grid relay
    {0x52000200, 0x00237700, 0x002377FF},  // inverter temperature
};

static inline uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t get32(const uint8_t *p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
static inline uint64_t get64(const uint8_t *p) { return uint64_t(get32(p)) | (uint64_t(get32(p + 4)) << 32); }
static inline void put16(std::vector<uint8_t> &v, uint16_t x) {
  v.push_back(x & 0xFF);
  v.push_back(x >> 8);
}
static inline void put32(std::vector<uint8_t> &v, uint32_t x) {
  put16(v, x & 0xFFFF);
  put16(v, x >> 16);
}

// PPP FCS-16 (RFC 1662), as used by SMANET2+ over Bluetooth.
static uint16_t fcs16(const uint8_t *data, size_t len) {
  uint16_t fcs = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    fcs ^= data[i];
    for (int b = 0; b < 8; b++)
      fcs = (fcs & 1) ? (fcs >> 1) ^ 0x8408 : (fcs >> 1);
  }
  return fcs ^ 0xFFFF;
}

static inline bool needs_escape(uint8_t b) { return b == 0x7D || b == 0x7E || b == 0x11 || b == 0x12 || b == 0x13; }

static void publish(sensor::Sensor *s, float value) {
  if (s != nullptr)
    s->publish_state(value);
}
static void publish(text_sensor::TextSensor *s, const std::string &value) {
  if (s != nullptr)
    s->publish_state(value);
}

// Returns the selected tag of a 40-byte status record, or 0 when none is selected.
static uint32_t selected_attribute(const uint8_t *rec, size_t rec_size) {
  for (size_t i = 8; i + 4 <= rec_size; i += 4) {
    uint32_t attr = get32(rec + i);
    uint32_t tag = attr & 0x00FFFFFF;
    if (tag == 0xFFFFFE)
      break;
    if ((attr >> 24) == 1)
      return tag;
  }
  return 0;
}

static std::string status_to_string(uint32_t tag) {
  switch (tag) {
    case 35:
      return "Fault";
    case 303:
      return "Off";
    case 307:
      return "Ok";
    case 455:
      return "Warning";
    case 51:
      return "Closed";
    case 311:
      return "Open";
    case 16777213:
      return "Not available";
    default:
      return str_sprintf("Unknown (%" PRIu32 ")", tag);
  }
}

static std::string version_to_string(uint32_t v) {
  uint8_t type = v & 0xFF;
  uint8_t build = (v >> 8) & 0xFF;
  uint8_t minor = (v >> 16) & 0xFF;
  uint8_t major = (v >> 24) & 0xFF;
  char t = type > 5 ? '?' : "NEABRS"[type];
  // major/minor are BCD encoded
  return str_sprintf("%c%c.%c%c.%02u.%c", '0' + (major >> 4), '0' + (major & 0x0F), '0' + (minor >> 4),
                     '0' + (minor & 0x0F), build, t);
}

void SmaBluetooth::set_address(uint64_t address) {
  for (int i = 0; i < 6; i++) {
    this->peer_bda_[i] = (address >> (40 - 8 * i)) & 0xFF;
    this->inverter_addr_[5 - i] = this->peer_bda_[i];
  }
}

// ---------------------------------------------------------------------------
// Bluetooth stack
// ---------------------------------------------------------------------------

void SmaBluetooth::setup() {
  instance_ = this;
  this->rx_stream_ = xStreamBufferCreate(4096, 1);
  this->app_serial_ = 900000000 + random_uint32() % 100000000;
  if (this->rx_stream_ == nullptr || !this->init_bluetooth_()) {
    this->mark_failed();
    return;
  }
  if (this->connected_binary_sensor_ != nullptr)
    this->connected_binary_sensor_->publish_initial_state(false);
}

bool SmaBluetooth::init_bluetooth_() {
  esp_err_t err;
  // BLE is not used; give its controller memory back to the heap.
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&cfg)) != ESP_OK) {
      ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
      return false;
    }
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
      ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
      return false;
    }
  }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    err = esp_bluedroid_init_with_cfg(&bd_cfg);
#else
    err = esp_bluedroid_init();
#endif
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(err));
      return false;
    }
  }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
    if ((err = esp_bluedroid_enable()) != ESP_OK) {
      ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(err));
      return false;
    }
  }
  if ((err = esp_bt_gap_register_callback(SmaBluetooth::gap_callback_)) != ESP_OK) {
    ESP_LOGE(TAG, "GAP callback registration failed: %s", esp_err_to_name(err));
    return false;
  }
  if ((err = esp_spp_register_callback(SmaBluetooth::spp_callback_)) != ESP_OK) {
    ESP_LOGE(TAG, "SPP callback registration failed: %s", esp_err_to_name(err));
    return false;
  }
  esp_spp_cfg_t spp_cfg = {};
  spp_cfg.mode = ESP_SPP_MODE_CB;
  spp_cfg.enable_l2cap_ertm = true;
  spp_cfg.tx_buffer_size = 0;
  if ((err = esp_spp_enhanced_init(&spp_cfg)) != ESP_OK) {
    ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(err));
    return false;
  }
  // SMA inverters use legacy pairing with PIN 0000 when they ask for it at all.
  esp_bt_pin_code_t pin;
  memcpy(pin, BT_PIN, sizeof(BT_PIN));
  esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, sizeof(BT_PIN), pin);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  for (int i = 0; i < 6; i++)
    this->local_addr_[5 - i] = mac[i];
  return true;
}

void SmaBluetooth::gap_callback_(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  if (event == ESP_BT_GAP_PIN_REQ_EVT) {
    esp_bt_pin_code_t pin;
    memcpy(pin, BT_PIN, sizeof(BT_PIN));
    esp_bt_gap_pin_reply(param->pin_req.bda, true, sizeof(BT_PIN), pin);
  }
}

// Runs in the Bluedroid task: only touch atomics and the stream buffer here.
void SmaBluetooth::spp_callback_(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  SmaBluetooth *self = instance_;
  if (self == nullptr)
    return;
  switch (event) {
    case ESP_SPP_INIT_EVT:
      if (param->init.status == ESP_SPP_SUCCESS)
        self->spp_ready_ = true;
      else
        self->last_status_ = param->init.status;
      break;
    case ESP_SPP_CL_INIT_EVT:
      if (param->cl_init.status != ESP_SPP_SUCCESS) {
        self->last_status_ = param->cl_init.status;
        self->ev_open_failed_ = true;
      }
      break;
    case ESP_SPP_OPEN_EVT:
      if (param->open.status == ESP_SPP_SUCCESS) {
        self->spp_handle_ = param->open.handle;
        self->ev_open_ = true;
      } else {
        self->last_status_ = param->open.status;
        self->ev_open_failed_ = true;
      }
      break;
    case ESP_SPP_CLOSE_EVT:
      self->spp_handle_ = 0;
      self->ev_closed_ = true;
      break;
    case ESP_SPP_DATA_IND_EVT:
      xStreamBufferSend(self->rx_stream_, param->data_ind.data, param->data_ind.len, 0);
      break;
    case ESP_SPP_WRITE_EVT:
      self->tx_congested_ = param->write.cong;
      self->tx_busy_ = false;
      break;
    case ESP_SPP_CONG_EVT:
      self->tx_congested_ = param->cong.cong;
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Main loop / connection state machine
// ---------------------------------------------------------------------------

void SmaBluetooth::set_state_(State state) {
  this->state_ = state;
  this->state_since_ = millis();
}

void SmaBluetooth::loop() {
  if (this->state_ == State::BT_INIT) {
    if (this->spp_ready_) {
      ESP_LOGI(TAG, "Bluetooth SPP ready");
      this->set_state_(State::IDLE);
    } else if (this->last_status_ != 0) {
      ESP_LOGE(TAG, "SPP init failed (status %d)", this->last_status_.load());
      this->mark_failed();
    }
    return;
  }

  this->handle_bt_events_();
  this->process_rx_();
  this->flush_tx_();

  const uint32_t now = millis();
  const uint32_t elapsed = now - this->state_since_;
  switch (this->state_) {
    case State::IDLE:
      if (int32_t(now - this->next_attempt_) >= 0)
        this->start_connect_();
      break;
    case State::CONNECTING:
      if (elapsed > CONNECT_TIMEOUT)
        this->disconnect_("connect timeout");
      break;
    case State::WAIT_HELLO:
      if (elapsed > HELLO_TIMEOUT)
        this->disconnect_("no announcement from inverter");
      break;
    case State::WAIT_NET_READY:
      if (elapsed > NET_READY_TIMEOUT) {
        // Some firmwares skip the topology packet; carry on once we have seen 0x000A.
        if (this->got_net_info_)
          this->send_discovery_();
        else
          this->disconnect_("network setup timeout");
      }
      break;
    case State::WAIT_DISCOVERY:
      if (elapsed > REPLY_TIMEOUT)
        this->disconnect_("no identification reply");
      break;
    case State::WAIT_LOGON:
      if (elapsed > REPLY_TIMEOUT)
        this->disconnect_("no logon reply");
      break;
    case State::READY:
      if (this->poll_requested_) {
        this->poll_requested_ = false;
        this->start_poll_cycle_();
      }
      break;
    case State::WAIT_REPLY:
      if (elapsed > REPLY_TIMEOUT) {
        const Query &q = this->queries_[this->query_index_];
        ESP_LOGW(TAG, "Timeout waiting for query 0x%08" PRIX32 " [0x%08" PRIX32 "]", q.command, q.first);
        if (++this->consecutive_timeouts_ >= MAX_CONSECUTIVE_TIMEOUTS) {
          this->disconnect_("too many query timeouts");
        } else {
          this->query_index_++;
          this->send_next_query_();
        }
      }
      break;
    default:
      break;
  }
}

void SmaBluetooth::handle_bt_events_() {
  if (this->ev_open_failed_.exchange(false) && this->state_ == State::CONNECTING) {
    ESP_LOGW(TAG, "Connection to inverter failed (status %d), retrying in %" PRIu32 " s", this->last_status_.load(),
             this->reconnect_interval_ / 1000);
    this->on_disconnected_();
  }
  if (this->ev_open_.exchange(false)) {
    if (this->state_ == State::CONNECTING) {
      ESP_LOGI(TAG, "Bluetooth connected, starting SMA handshake");
      this->set_state_(State::WAIT_HELLO);
      this->send_l1_(CMD_VERSION, ADDR_VERSION, {'v', 'e', 'r', '\r', '\n'});
    } else if (this->spp_handle_ != 0) {
      esp_spp_disconnect(this->spp_handle_);  // late connect after we gave up
    }
  }
  if (this->ev_closed_.exchange(false) && this->state_ != State::IDLE) {
    if (this->state_ == State::CONNECTING) {
      // Bluedroid reports a failed page as a close; usually out of range or inverter asleep.
      ESP_LOGW(TAG, "Inverter not reachable over Bluetooth, retrying in %" PRIu32 " s", this->reconnect_interval_ / 1000);
    } else {
      ESP_LOGW(TAG, "Bluetooth connection closed by inverter");
    }
    this->on_disconnected_();
  }
}

void SmaBluetooth::start_connect_() {
  this->rx_buf_.clear();
  this->l2_buf_.clear();
  this->l2_escape_ = false;
  this->tx_queue_.clear();
  this->tx_busy_ = false;
  this->tx_congested_ = false;
  this->got_net_info_ = false;
  this->ev_open_ = false;
  this->ev_open_failed_ = false;
  this->ev_closed_ = false;
  xStreamBufferReset(this->rx_stream_);

  ESP_LOGD(TAG, "Connecting to %02X:%02X:%02X:%02X:%02X:%02X", this->peer_bda_[0], this->peer_bda_[1],
           this->peer_bda_[2], this->peer_bda_[3], this->peer_bda_[4], this->peer_bda_[5]);
  esp_err_t err = esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, SPP_CHANNEL, this->peer_bda_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_spp_connect failed: %s", esp_err_to_name(err));
    this->next_attempt_ = millis() + this->reconnect_interval_;
    return;
  }
  this->set_state_(State::CONNECTING);
}

void SmaBluetooth::disconnect_(const char *reason) {
  ESP_LOGW(TAG, "Disconnecting: %s", reason);
  uint32_t handle = this->spp_handle_;
  if (handle != 0)
    esp_spp_disconnect(handle);
  this->on_disconnected_();
}

void SmaBluetooth::on_disconnected_() {
  bool was_online = this->state_ == State::READY || this->state_ == State::WAIT_REPLY;
  this->set_state_(State::IDLE);
  this->next_attempt_ = millis() + this->reconnect_interval_;
  this->tx_queue_.clear();
  this->tx_busy_ = false;
  if (this->connected_binary_sensor_ != nullptr)
    this->connected_binary_sensor_->publish_state(false);
  if (was_online || this->ac_power_sensor_ == nullptr || !this->ac_power_sensor_->has_state())
    this->publish_offline_();
}

void SmaBluetooth::update() {
  if (this->state_ == State::READY) {
    this->start_poll_cycle_();
  } else if (this->state_ == State::WAIT_REPLY) {
    ESP_LOGD(TAG, "Previous poll still running, skipping");
  }
}

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------

void SmaBluetooth::process_rx_() {
  uint8_t tmp[256];
  size_t n;
  while ((n = xStreamBufferReceive(this->rx_stream_, tmp, sizeof(tmp), 0)) > 0)
    this->rx_buf_.insert(this->rx_buf_.end(), tmp, tmp + n);

  size_t pos = 0;
  while (this->rx_buf_.size() - pos >= 4 && this->state_ != State::IDLE) {
    const uint8_t *p = &this->rx_buf_[pos];
    uint16_t len = p[1] | (p[2] << 8);
    if (p[0] != 0x7E || (p[0] ^ p[1] ^ p[2]) != p[3] || len < L1_HEADER_SIZE || len > 1024) {
      pos++;  // resync
      continue;
    }
    if (this->rx_buf_.size() - pos < len)
      break;
    this->handle_l1_frame_(p, len);
    pos += len;
  }
  if (this->state_ == State::IDLE) {
    this->rx_buf_.clear();
  } else {
    this->rx_buf_.erase(this->rx_buf_.begin(), this->rx_buf_.begin() + pos);
  }
}

void SmaBluetooth::handle_l1_frame_(const uint8_t *frame, size_t len) {
  const uint16_t cmd = get16(frame + 16);
  const uint8_t *payload = frame + L1_HEADER_SIZE;
  const size_t plen = len - L1_HEADER_SIZE;

  if (cmd != CMD_L2_LAST && cmd != CMD_L2_PART) {
    this->handle_l1_(cmd, payload, plen);
    return;
  }

  // Start of a new L2 packet discards any unfinished one.
  if (plen >= 5 && payload[0] == 0x7E && get32(payload + 1) == L2_SIGNATURE) {
    this->l2_buf_.clear();
    this->l2_escape_ = false;
  }
  for (size_t i = 0; i < plen; i++) {
    uint8_t b = payload[i];
    if (this->l2_escape_) {
      this->l2_buf_.push_back(b ^ 0x20);
      this->l2_escape_ = false;
    } else if (b == 0x7D) {
      this->l2_escape_ = true;
    } else {
      this->l2_buf_.push_back(b);
    }
  }
  if (this->l2_buf_.size() > 2048) {
    ESP_LOGW(TAG, "L2 packet too large, dropping");
    this->l2_buf_.clear();
    return;
  }
  if (cmd == CMD_L2_PART)
    return;

  std::vector<uint8_t> pkt;
  pkt.swap(this->l2_buf_);
  this->l2_escape_ = false;
  const size_t n = pkt.size();
  if (n < 36 || pkt[0] != 0x7E || pkt[n - 1] != 0x7E || get32(&pkt[1]) != L2_SIGNATURE) {
    ESP_LOGV(TAG, "Ignoring malformed L2 packet (%u bytes)", (unsigned) n);
    return;
  }
  if (fcs16(&pkt[1], n - 4) != get16(&pkt[n - 3])) {
    ESP_LOGW(TAG, "L2 checksum mismatch, dropping packet");
    return;
  }
  this->handle_l2_(pkt);
}

void SmaBluetooth::handle_l1_(uint16_t cmd, const uint8_t *payload, size_t len) {
  ESP_LOGV(TAG, "L1 cmd 0x%04X, %u bytes", cmd, (unsigned) len);
  switch (cmd) {
    case CMD_HELLO:
      if (this->state_ == State::WAIT_HELLO && len >= 5) {
        if (payload[1] < 4)
          ESP_LOGW(TAG, "Inverter BT protocol version %u; firmware < 1.71 is not supported", payload[1]);
        this->net_id_ = payload[4];
        ESP_LOGD(TAG, "Inverter announced, NetID %u", this->net_id_);
        this->send_hello_reply_();
        this->set_state_(State::WAIT_NET_READY);
      }
      break;
    case CMD_NET_INFO:
      if (this->state_ == State::WAIT_NET_READY && len >= 13) {
        // payload: root device address[6], flag, local address[6]
        memcpy(this->local_addr_, payload + 7, 6);
        this->got_net_info_ = true;
      }
      break;
    case CMD_TOPOLOGY:
      if (this->state_ == State::WAIT_NET_READY)
        this->send_discovery_();
      break;
    case CMD_SIGNAL_REPLY:
      if (len >= 5)
        publish(this->bluetooth_signal_sensor_, payload[4] * 100.0f / 255.0f);
      break;
    default:
      break;
  }
}

void SmaBluetooth::flush_tx_() {
  if (this->tx_busy_ && millis() - this->tx_since_ > 2000)
    this->tx_busy_ = false;  // never got ESP_SPP_WRITE_EVT
  if (this->tx_queue_.empty() || this->tx_busy_ || this->tx_congested_)
    return;
  uint32_t handle = this->spp_handle_;
  if (handle == 0) {
    this->tx_queue_.clear();
    return;
  }
  std::vector<uint8_t> &frame = this->tx_queue_.front();
  this->tx_busy_ = true;
  this->tx_since_ = millis();
  esp_err_t err = esp_spp_write(handle, frame.size(), frame.data());
  this->tx_queue_.pop_front();
  if (err != ESP_OK) {
    this->tx_busy_ = false;
    ESP_LOGW(TAG, "esp_spp_write failed: %s", esp_err_to_name(err));
  }
}

void SmaBluetooth::send_l1_(uint16_t cmd, const uint8_t *dst, const std::vector<uint8_t> &payload) {
  const size_t len = L1_HEADER_SIZE + payload.size();
  std::vector<uint8_t> f;
  f.reserve(len);
  f.push_back(0x7E);
  f.push_back(len & 0xFF);
  f.push_back(len >> 8);
  f.push_back(f[0] ^ f[1] ^ f[2]);
  f.insert(f.end(), this->local_addr_, this->local_addr_ + 6);
  f.insert(f.end(), dst, dst + 6);
  put16(f, cmd);
  f.insert(f.end(), payload.begin(), payload.end());
  this->tx_queue_.push_back(std::move(f));
  this->flush_tx_();
}

uint16_t SmaBluetooth::send_l2_(uint8_t ctrl, uint16_t ctrl2, uint16_t dst_susy, uint32_t dst_serial,
                                const std::vector<uint8_t> &data) {
  // The FCS is sent unescaped, so (like SBFspot) bump the packet id until it contains no 7D/7E.
  for (int attempt = 0; attempt < 32; attempt++) {
    this->packet_id_ = (this->packet_id_ + 1) & 0x7FFF;
    if (this->packet_id_ == 0)
      this->packet_id_ = 1;

    std::vector<uint8_t> body;
    body.reserve(28 + data.size());
    put32(body, L2_SIGNATURE);
    body.push_back((24 + data.size()) / 4);  // length in 32-bit words after the signature
    body.push_back(ctrl);
    put16(body, dst_susy);
    put32(body, dst_serial);
    put16(body, ctrl2);
    put16(body, APP_SUSY_ID);
    put32(body, this->app_serial_);
    put16(body, ctrl2);
    put16(body, 0);  // error code
    put16(body, 0);  // fragment counter
    put16(body, this->packet_id_ | 0x8000);
    body.insert(body.end(), data.begin(), data.end());

    const uint16_t fcs = fcs16(body.data(), body.size());
    const uint8_t lo = fcs & 0xFF, hi = fcs >> 8;
    if (lo == 0x7E || lo == 0x7D || hi == 0x7E || hi == 0x7D)
      continue;

    std::vector<uint8_t> payload;
    payload.reserve(body.size() * 2 + 4);
    payload.push_back(0x7E);
    for (uint8_t b : body) {
      if (needs_escape(b)) {
        payload.push_back(0x7D);
        payload.push_back(b ^ 0x20);
      } else {
        payload.push_back(b);
      }
    }
    payload.push_back(lo);
    payload.push_back(hi);
    payload.push_back(0x7E);
    this->send_l1_(CMD_L2_LAST, ADDR_UNKNOWN, payload);
    return this->packet_id_;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Protocol steps
// ---------------------------------------------------------------------------

void SmaBluetooth::send_hello_reply_() {
  std::vector<uint8_t> p;
  put32(p, 0x00700400);
  p.push_back(this->net_id_);
  put32(p, 0);
  put32(p, 1);
  this->send_l1_(CMD_HELLO, this->inverter_addr_, p);
}

void SmaBluetooth::send_discovery_() {
  std::vector<uint8_t> d;
  put32(d, 0x00000200);
  put32(d, 0);
  put32(d, 0);
  this->expected_packet_id_ = this->send_l2_(0xA0, 0x0000, 0xFFFF, 0xFFFFFFFF, d);
  this->set_state_(State::WAIT_DISCOVERY);
}

void SmaBluetooth::send_logoff_() {
  std::vector<uint8_t> d;
  put32(d, 0xFFFD010E);
  put32(d, 0xFFFFFFFF);
  this->send_l2_(0xA0, 0x0300, 0xFFFF, 0xFFFFFFFF, d);
}

void SmaBluetooth::send_logon_() {
  const uint8_t enc = this->user_group_ == 0x07 ? 0x88 : 0xBB;
  time_t t = ::time(nullptr);
  this->logon_time_ = t > 0 ? uint32_t(t) : millis() / 1000 + 1;

  std::vector<uint8_t> d;
  put32(d, 0xFFFD040C);
  put32(d, this->user_group_);
  put32(d, 900);  // session timeout in seconds
  put32(d, this->logon_time_);
  put32(d, 0);
  for (size_t i = 0; i < 12; i++)
    d.push_back(i < this->password_.size() ? uint8_t(this->password_[i] + enc) : enc);
  this->expected_packet_id_ = this->send_l2_(0xA0, 0x0100, 0xFFFF, 0xFFFFFFFF, d);
  this->set_state_(State::WAIT_LOGON);
}

void SmaBluetooth::send_signal_request_() { this->send_l1_(CMD_SIGNAL_REQUEST, this->inverter_addr_, {0x05, 0x00}); }

void SmaBluetooth::start_poll_cycle_() {
  if (millis() - this->logon_millis_ > RELOGON_INTERVAL) {
    ESP_LOGD(TAG, "Refreshing logon session");
    this->send_logon_();
    return;
  }
  this->queries_.clear();
  if (!this->info_fetched_)
    this->queries_.insert(this->queries_.end(), std::begin(INFO_QUERIES), std::end(INFO_QUERIES));
  this->queries_.insert(this->queries_.end(), std::begin(SPOT_QUERIES), std::end(SPOT_QUERIES));
  this->query_index_ = 0;
  this->dc_power_[0] = this->dc_power_[1] = NAN;
  if (this->bluetooth_signal_sensor_ != nullptr)
    this->send_signal_request_();
  this->send_next_query_();
}

void SmaBluetooth::send_next_query_() {
  if (this->query_index_ >= this->queries_.size()) {
    this->info_fetched_ = true;
    if (!std::isnan(this->dc_power_[0]) || !std::isnan(this->dc_power_[1])) {
      float total = (std::isnan(this->dc_power_[0]) ? 0 : this->dc_power_[0]) +
                    (std::isnan(this->dc_power_[1]) ? 0 : this->dc_power_[1]);
      publish(this->dc_power_sensor_, total);
    }
    this->set_state_(State::READY);
    return;
  }
  const Query &q = this->queries_[this->query_index_];
  std::vector<uint8_t> d;
  put32(d, q.command);
  put32(d, q.first);
  put32(d, q.last);
  this->expected_packet_id_ = this->send_l2_(0xA0, 0x0000, this->inverter_susy_, this->inverter_serial_, d);
  this->set_state_(State::WAIT_REPLY);
}

void SmaBluetooth::handle_l2_(const std::vector<uint8_t> &pkt) {
  const size_t n = pkt.size();
  const uint16_t src_susy = get16(&pkt[15]);
  const uint32_t src_serial = get32(&pkt[17]);
  const uint16_t error = get16(&pkt[23]);
  const uint16_t fragments_left = get16(&pkt[25]);
  const uint16_t packet_id = get16(&pkt[27]) & 0x7FFF;

  if (packet_id != this->expected_packet_id_) {
    ESP_LOGV(TAG, "Ignoring L2 packet id %u (expected %u)", packet_id, this->expected_packet_id_);
    return;
  }

  switch (this->state_) {
    case State::WAIT_DISCOVERY:
      if (n >= 61) {
        this->inverter_susy_ = get16(&pkt[55]);
        this->inverter_serial_ = get32(&pkt[57]);
      } else {
        this->inverter_susy_ = src_susy;
        this->inverter_serial_ = src_serial;
      }
      ESP_LOGD(TAG, "Inverter SUSyID %u, serial %" PRIu32, this->inverter_susy_, this->inverter_serial_);
      this->send_logoff_();
      this->send_logon_();
      break;

    case State::WAIT_LOGON:
      if (n < 45 || get32(&pkt[41]) != this->logon_time_)
        return;
      if (error == 0x0100) {
        ESP_LOGE(TAG, "Logon rejected: wrong password for this user group");
        this->disconnect_("invalid password");
        return;
      }
      if (error != 0) {
        ESP_LOGW(TAG, "Logon failed with error 0x%04X", error);
        this->disconnect_("logon failed");
        return;
      }
      this->inverter_susy_ = src_susy;
      this->inverter_serial_ = src_serial;
      this->logon_millis_ = millis();
      this->consecutive_timeouts_ = 0;
      ESP_LOGI(TAG, "Logged on to inverter, serial %" PRIu32, src_serial);
      publish(this->serial_number_text_sensor_, to_string(src_serial));
      if (this->connected_binary_sensor_ != nullptr)
        this->connected_binary_sensor_->publish_state(true);
      this->set_state_(State::READY);
      this->poll_requested_ = true;
      break;

    case State::WAIT_REPLY: {
      this->consecutive_timeouts_ = 0;
      const Query &q = this->queries_[this->query_index_];
      if (error != 0) {
        // e.g. 0x0015: value not supported by this model / not available right now
        ESP_LOGD(TAG, "Query 0x%08" PRIX32 " [0x%08" PRIX32 "] returned error 0x%04X", q.command, q.first, error);
      } else {
        this->parse_records_(pkt);
      }
      if (error == 0 && fragments_left != 0) {
        this->state_since_ = millis();  // more packets for this query follow
        return;
      }
      this->query_index_++;
      this->send_next_query_();
      break;
    }

    default:
      break;
  }
}

void SmaBluetooth::parse_records_(const std::vector<uint8_t> &pkt) {
  const size_t end = pkt.size() - 3;  // strip FCS + trailing 7E
  if (end <= L2_DATA_OFFSET)
    return;
  const uint32_t first = get32(&pkt[33]);
  const uint32_t last = get32(&pkt[37]);
  if (last < first)
    return;
  const size_t count = size_t(last - first) + 1;
  const size_t rec_size = (end - L2_DATA_OFFSET) / count;
  if (rec_size < 16) {
    ESP_LOGW(TAG, "Unexpected record size %u", (unsigned) rec_size);
    return;
  }

  for (size_t i = L2_DATA_OFFSET; i + rec_size <= end; i += rec_size) {
    const uint8_t *r = &pkt[i];
    const uint32_t code = get32(r);
    const uint32_t lri = code & 0x00FFFF00;
    const uint8_t cls = code & 0xFF;
    const uint8_t type = code >> 24;

    bool nan = false;
    int64_t v64 = 0;
    int32_t v32 = 0;
    if (rec_size == 16) {
      uint64_t raw = get64(r + 8);
      nan = raw == 0x8000000000000000ULL || raw == 0xFFFFFFFFFFFFFFFFULL;
      v64 = int64_t(raw);
    } else if (type != DT_STRING && type != DT_STATUS) {
      uint32_t raw = get32(r + 16);
      nan = raw == 0x80000000UL || raw == 0xFFFFFFFFUL;
      v32 = int32_t(raw);
    }
    // Spot values are NaN while the inverter is not feeding in; report them as 0 like SBFspot.
    const float spot = nan ? 0.0f : float(v32);
    ESP_LOGV(TAG, "Record 0x%08" PRIX32 " value %" PRId32 " nan=%d", code, v32, nan);

    switch (lri) {
      case LRI_AC_POWER_TOTAL:
        publish(this->ac_power_sensor_, spot);
        break;
      case LRI_AC_VOLTAGE_A:
        publish(this->ac_voltage_sensor_, spot / 100.0f);
        break;
      case LRI_AC_CURRENT_A_1:
        publish(this->ac_current_sensor_, spot / 1000.0f);
        break;
      case LRI_AC_CURRENT_A:
        // Alternative current record on some models; must not overwrite a valid _1 value with 0.
        if (!nan)
          publish(this->ac_current_sensor_, spot / 1000.0f);
        break;
      case LRI_GRID_FREQUENCY:
        if (!nan)
          publish(this->grid_frequency_sensor_, spot / 100.0f);
        break;
      case LRI_DC_POWER:
        if (cls == 1 || cls == 2) {
          this->dc_power_[cls - 1] = spot;
          publish(cls == 1 ? this->dc_power_a_sensor_ : this->dc_power_b_sensor_, spot);
        }
        break;
      case LRI_DC_VOLTAGE:
        if (cls == 1 || cls == 2)
          publish(cls == 1 ? this->dc_voltage_a_sensor_ : this->dc_voltage_b_sensor_, spot / 100.0f);
        break;
      case LRI_DC_CURRENT:
        if (cls == 1 || cls == 2)
          publish(cls == 1 ? this->dc_current_a_sensor_ : this->dc_current_b_sensor_, spot / 1000.0f);
        break;
      case LRI_ENERGY_TOTAL:
        // Never publish a fake 0 for a total_increasing counter.
        if (!nan && v64 > 0)
          publish(this->energy_total_sensor_, v64 / 1000.0f);
        break;
      case LRI_ENERGY_TODAY:
        if (!nan)
          publish(this->energy_today_sensor_, v64 / 1000.0f);
        break;
      case LRI_OPERATING_TIME:
        if (!nan)
          publish(this->operating_time_sensor_, v64 / 3600.0f);
        break;
      case LRI_FEED_IN_TIME:
        if (!nan)
          publish(this->feed_in_time_sensor_, v64 / 3600.0f);
        break;
      case LRI_TEMPERATURE:
        if (!nan)
          publish(this->temperature_sensor_, v32 / 100.0f);
        break;
      case LRI_OPERATION_HEALTH:
        if (uint32_t tag = selected_attribute(r, rec_size))
          publish(this->device_status_text_sensor_, status_to_string(tag));
        break;
      case LRI_GRID_RELAY:
        if (uint32_t tag = selected_attribute(r, rec_size))
          publish(this->grid_relay_text_sensor_, status_to_string(tag));
        break;
      case LRI_DEVICE_NAME: {
        const char *s = reinterpret_cast<const char *>(r + 8);
        publish(this->device_name_text_sensor_, std::string(s, strnlen(s, rec_size - 8)));
        break;
      }
      case LRI_SOFTWARE_VERSION:
        if (rec_size >= 28)
          publish(this->software_version_text_sensor_, version_to_string(get32(r + 24)));
        break;
      default:
        break;
    }
  }
}

void SmaBluetooth::publish_offline_() {
  // Inverters switch Bluetooth off at night, so "offline" usually just means "no sun".
  const float zero = this->zero_when_offline_ ? 0.0f : NAN;
  publish(this->ac_power_sensor_, zero);
  publish(this->ac_current_sensor_, zero);
  publish(this->dc_power_sensor_, zero);
  publish(this->dc_power_a_sensor_, zero);
  publish(this->dc_power_b_sensor_, zero);
  publish(this->dc_current_a_sensor_, zero);
  publish(this->dc_current_b_sensor_, zero);
  publish(this->ac_voltage_sensor_, NAN);
  publish(this->dc_voltage_a_sensor_, NAN);
  publish(this->dc_voltage_b_sensor_, NAN);
  publish(this->grid_frequency_sensor_, NAN);
  publish(this->temperature_sensor_, NAN);
  publish(this->bluetooth_signal_sensor_, NAN);
  publish(this->device_status_text_sensor_, "Offline");
}

void SmaBluetooth::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SMA Bluetooth:\n"
                "  Inverter address: %02X:%02X:%02X:%02X:%02X:%02X\n"
                "  User group: %s\n"
                "  Reconnect interval: %" PRIu32 " s\n"
                "  Zero when offline: %s",
                this->peer_bda_[0], this->peer_bda_[1], this->peer_bda_[2], this->peer_bda_[3], this->peer_bda_[4],
                this->peer_bda_[5], this->user_group_ == 0x07 ? "user" : "installer", this->reconnect_interval_ / 1000,
                YESNO(this->zero_when_offline_));
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "AC Power", this->ac_power_sensor_);
  LOG_SENSOR("  ", "Energy Today", this->energy_today_sensor_);
  LOG_SENSOR("  ", "Energy Total", this->energy_total_sensor_);
  LOG_TEXT_SENSOR("  ", "Device Status", this->device_status_text_sensor_);
  LOG_BINARY_SENSOR("  ", "Connected", this->connected_binary_sensor_);
}

}  // namespace sma_bluetooth
}  // namespace esphome
