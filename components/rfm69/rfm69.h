#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef ESPHOME_LOG_HAS_CONFIG
#include "esphome/core/log.h"
#endif

// Для тестування використовуємо mock SPI, для ESPHome - звичайний
#ifdef ARDUINO
#include <SPI.h>
#else
#include "esphome/components/spi/spi.h"
#endif

#include <string>
#include <memory>
#include <cmath>
#include <cstring>

namespace esphome {
namespace rfm69 {

// RFM69 Constants from MySensors driver
#define RFM69_FIFO_SIZE                  (0xFFu)
#define RFM69_MAX_PACKET_LEN             (0x40u)  // 64 bytes
#define RFM69_PACKET_HEADER_VERSION      (1u)
#define RFM69_MIN_PACKET_HEADER_VERSION  (1u)

#define RFM69_RETRIES                    (5u)
#define RFM69_RETRY_TIMEOUT_MS           (200ul)
#define RFM69_MODE_READY_TIMEOUT_MS      (50ul)

#define RFM69_ACK_REQUESTED              (7u)
#define RFM69_ACK_RECEIVED               (6u)
#define RFM69_ACK_RSSI_REPORT            (5u)

#define RFM69_BROADCAST_ADDRESS          (255u)
#define RFM69_TARGET_RSSI_DBM            (-75)
#define RFM69_HIGH_POWER_DBM             (18u)

// Frequency constants
#define RFM69_315MHZ                     (315000000ul)
#define RFM69_433MHZ                     (433920000ul)
#define RFM69_865MHZ                     (865500000ul)
#define RFM69_868MHZ                     (868000000ul)
#define RFM69_915MHZ                     (915000000ul)

#define RFM69_FXOSC                      (32*1000000ul)
#define RFM69_FSTEP                      (RFM69_FXOSC / 524288.0f)

// Power level ranges
#ifdef MY_RFM69HW
#define RFM69_MIN_POWER_LEVEL_DBM        (-2)
#define RFM69_MAX_POWER_LEVEL_DBM        (20)
#else
#define RFM69_MIN_POWER_LEVEL_DBM        (-18)
#define RFM69_MAX_POWER_LEVEL_DBM        (13)
#endif

// Radio modes
enum class RFM69RadioMode : uint8_t {
    RX = 0,
    TX = 1,
    CAD = 2,
    SLEEP = 3,
    STDBY = 4,
    SYNTH = 5,
    LISTEN = 6
};

// Data types
using RFM69SequenceNumber = uint8_t;
using RFM69RSSI = uint8_t;
using RFM69ControlFlags = uint8_t;
using RFM69PowerLevel = int8_t;

// RFM69 header structure
struct __attribute__((packed)) RFM69Header {
    uint8_t packet_len;
    uint8_t recipient;
    uint8_t version;
    uint8_t sender;
    RFM69ControlFlags control_flags;
    RFM69SequenceNumber sequence_number;
};

// ACK packet structure
struct __attribute__((packed)) RFM69Ack {
    RFM69SequenceNumber sequence_number;
    RFM69RSSI rssi;
};

// Packet structure
struct __attribute__((packed)) RFM69Packet {
    union {
        struct {
            RFM69Header header;
            union {
                uint8_t payload[RFM69_MAX_PACKET_LEN - sizeof(RFM69Header)];
                RFM69Ack ack;
            };
        };
        uint8_t data[RFM69_MAX_PACKET_LEN];
    };
    uint8_t payload_len;
    RFM69RSSI rssi;
};

class RFM69Component : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration methods
  void set_cs_pin(std::shared_ptr<GPIOPin> cs_pin) { this->cs_pin_ = cs_pin; }
  void set_irq_pin(std::shared_ptr<GPIOPin> irq_pin) { this->irq_pin_ = irq_pin; }
  void set_rst_pin(std::shared_ptr<GPIOPin> rst_pin) { this->rst_pin_ = rst_pin; }

  void set_frequency(float frequency) { this->frequency_ = frequency; }
  void set_network_id(uint8_t network_id) { this->network_id_ = network_id; }
  void set_node_id(uint8_t node_id) { this->node_id_ = node_id; }
  void set_high_power(bool high_power) { this->high_power_ = high_power; }
  void set_power_level(uint8_t power_level) { this->power_level_ = power_level; }
  void set_aes_key(const std::string &key) { this->aes_key_ = key; }

  // Public API methods
  bool send_message(uint8_t to_address, const std::string &data, bool no_ack = false);
  bool send_message(uint8_t to_address, const uint8_t *data, uint8_t len, bool no_ack = false);
  bool is_message_available();
  uint8_t receive_message(uint8_t *buffer, uint8_t max_len);

  // Power and mode control
  bool set_tx_power_level(RFM69PowerLevel power_level);
  RFM69PowerLevel get_tx_power_level() const { return this->power_level_dbm_; }
  uint8_t get_tx_power_percent() const;
  bool set_tx_power_percent(uint8_t power_percent);

  bool sleep();
  bool standby();
  bool set_radio_mode(RFM69RadioMode mode);

  // RSSI methods
  int16_t get_receiving_rssi() const;
  int16_t get_sending_rssi() const;

  // Statistics
  uint32_t get_packets_sent() const { return this->packets_sent_; }
  uint32_t get_packets_received() const { return this->packets_received_; }
  uint32_t get_packets_lost() const { return this->packets_lost_; }

  // Address management
  void set_address(uint8_t address) { this->node_id_ = address; }
  uint8_t get_address() const { return this->node_id_; }

 protected:
  // Hardware interface
  bool initialize_();
  bool sanity_check_();
  void interrupt_handler_();
  void handle_interrupt_();

  // Low-level register access
  uint8_t read_register_(uint8_t addr);
  void write_register_(uint8_t addr, uint8_t value);
  uint8_t burst_read_register_(uint8_t addr, uint8_t *buffer, uint8_t len);
  void burst_write_register_(uint8_t addr, const uint8_t *buffer, uint8_t len);

  // Packet handling
  bool send_frame_(RFM69Packet *packet, bool increase_sequence_counter = true);
  bool send_packet_(uint8_t recipient, const uint8_t *data, uint8_t len,
                   RFM69ControlFlags flags, bool increase_sequence_counter = true);
  void send_ack_(uint8_t recipient, RFM69SequenceNumber sequence_number, RFM69RSSI rssi);
  bool send_with_retry_(uint8_t recipient, const uint8_t *buffer, uint8_t buffer_size, bool no_ack);

  // Radio control
  void set_frequency_(uint32_t frequency_hz);
  void set_high_power_regs_(bool on_off);
  bool is_mode_ready_();
  void clear_fifo_();
  bool channel_free_();

  // RSSI and power management
  RFM69RSSI read_rssi_(bool force_trigger = false);
  int16_t rssi_internal_to_dbm_(RFM69RSSI internal_rssi) const;
  RFM69RSSI rssi_dbm_to_internal_(int16_t rssi_dbm) const;

  // Encryption
  void set_encryption_(const char *key);

  // Configuration
  void set_configuration_();

  // Helper macros implementation
  bool get_ack_requested_(RFM69ControlFlags flags) const { return (flags >> RFM69_ACK_REQUESTED) & 1; }
  void set_ack_requested_(RFM69ControlFlags &flags, bool value) {
    flags = (flags & ~(1 << RFM69_ACK_REQUESTED)) | (value << RFM69_ACK_REQUESTED);
  }
  bool get_ack_received_(RFM69ControlFlags flags) const { return (flags >> RFM69_ACK_RECEIVED) & 1; }
  void set_ack_received_(RFM69ControlFlags &flags, bool value) {
    flags = (flags & ~(1 << RFM69_ACK_RECEIVED)) | (value << RFM69_ACK_RECEIVED);
  }
  bool get_ack_rssi_report_(RFM69ControlFlags flags) const { return (flags >> RFM69_ACK_RSSI_REPORT) & 1; }
  void set_ack_rssi_report_(RFM69ControlFlags &flags, bool value) {
    flags = (flags & ~(1 << RFM69_ACK_RSSI_REPORT)) | (value << RFM69_ACK_RSSI_REPORT);
  }

 private:
  // GPIO pins
  std::shared_ptr<GPIOPin> cs_pin_;
  std::shared_ptr<GPIOPin> irq_pin_;
  std::shared_ptr<GPIOPin> rst_pin_;

  // Configuration
  float frequency_{868.0};
  uint8_t network_id_{100};
  uint8_t node_id_{1};
  bool high_power_{true};
  uint8_t power_level_{20};
  std::string aes_key_;

  // Internal state
  RFM69RadioMode radio_mode_{RFM69RadioMode::SLEEP};
  RFM69PowerLevel power_level_dbm_{13};
  RFM69SequenceNumber tx_sequence_number_{0};
  RFM69Packet current_packet_;

  // Flags
  volatile bool irq_flag_{false};
  bool data_received_{false};
  bool ack_received_{false};
  bool atc_enabled_{false};
  uint8_t atc_target_rssi_{0};

  // Statistics
  uint32_t packets_sent_{0};
  uint32_t packets_received_{0};
  uint32_t packets_lost_{0};
};

}  // namespace rfm69
}  // namespace esphome