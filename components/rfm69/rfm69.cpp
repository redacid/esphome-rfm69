#include "rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rfm69 {

static const char *const TAG = "rfm69";

// RFM69 регістри (базові)
#define RF_OPMODE 0x01
#define RF_DATAMODUL 0x02
#define RF_BITRATEMSB 0x03
#define RF_BITRATELSB 0x04
#define RF_FDEVMSB 0x05
#define RF_FDEVLSB 0x06
#define RF_FRFMSB 0x07
#define RF_FRFMID 0x08
#define RF_FRFLSB 0x09
#define RF_PALEVEL 0x11
#define RF_SYNCCONFIG 0x2E
#define RF_SYNCVALUE1 0x2F
#define RF_NODEADRS 0x39
#define RF_BROADCASTADRS 0x3A
#define RF_NETWORKID 0x39

#define RF_OPMODE_SLEEP 0x00
#define RF_OPMODE_STANDBY 0x04
#define RF_OPMODE_TRANSMITTER 0x0C
#define RF_OPMODE_RECEIVER 0x10

void RFM69Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RFM69...");
  
  // Ініціалізуємо піни
  this->cs_pin_->setup();
  this->cs_pin_->digital_write(true);
  
  this->irq_pin_->setup();
  
  if (this->rst_pin_ != nullptr) {
    this->rst_pin_->setup();
    // Ресет модуля
    this->rst_pin_->digital_write(true);
    delay(100);
    this->rst_pin_->digital_write(false);
    delay(100);
  }

  SPI.begin();
  
  if (!this->initialize_()) {
    ESP_LOGE(TAG, "Failed to initialize RFM69");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "RFM69 setup complete");
}

void RFM69Component::loop() {
  // Базова логіка прийому (якщо потрібно)
}

void RFM69Component::dump_config() {
  ESP_LOGCONFIG(TAG, "RFM69:");
  ESP_LOGCONFIG(TAG, "  Frequency: %.1f MHz", this->frequency_);
  ESP_LOGCONFIG(TAG, "  Network ID: %d", this->network_id_);
  ESP_LOGCONFIG(TAG, "  Node ID: %d", this->node_id_);
  ESP_LOGCONFIG(TAG, "  High Power: %s", this->high_power_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Power Level: %d", this->power_level_);
}

bool RFM69Component::initialize_() {
  // Базова ініціалізація RFM69
  this->write_register_(RF_OPMODE, RF_OPMODE_STANDBY);
  delay(10);
  
  // Перевіряємо зв'язок
  uint8_t version = this->read_register_(0x10);  // Version register
  if (version == 0x00 || version == 0xFF) {
    ESP_LOGE(TAG, "No RFM69 found! Version: 0x%02X", version);
    return false;
  }
  
  ESP_LOGI(TAG, "RFM69 version: 0x%02X", version);
  
  // Налаштування частоти (спрощено для 868 МГц)
  if (abs(this->frequency_ - 868.0) < 1.0) {
    this->write_register_(RF_FRFMSB, 0xD9);
    this->write_register_(RF_FRFMID, 0x00);
    this->write_register_(RF_FRFLSB, 0x00);
  }
  
  // Налаштування потужності
  if (this->high_power_) {
    this->write_register_(RF_PALEVEL, 0x60 | (this->power_level_ & 0x1F));
  } else {
    this->write_register_(RF_PALEVEL, 0x80 | (this->power_level_ & 0x1F));
  }
  
  // Network ID
  this->write_register_(RF_NETWORKID, this->network_id_);
  
  // Node address
  this->write_register_(RF_NODEADRS, this->node_id_);
  
  // Режим приймача
  this->write_register_(RF_OPMODE, RF_OPMODE_RECEIVER);
  
  return true;
}

void RFM69Component::write_register_(uint8_t addr, uint8_t value) {
  this->cs_pin_->digital_write(false);
  SPI.transfer(addr | 0x80);  // Write bit
  SPI.transfer(value);
  this->cs_pin_->digital_write(true);
}

uint8_t RFM69Component::read_register_(uint8_t addr) {
  this->cs_pin_->digital_write(false);
  SPI.transfer(addr & 0x7F);  // Read bit
  uint8_t value = SPI.transfer(0);
  this->cs_pin_->digital_write(true);
  return value;
}

bool RFM69Component::send_message(uint8_t to_address, const std::string &data) {
  ESP_LOGI(TAG, "Sending message to %d: %s", to_address, data.c_str());
  // Тут має бути логіка відправки
  return true;
}

}  // namespace rfm69
}  // namespace esphome