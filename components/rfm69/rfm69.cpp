#include "rfm69.h"
#include "rfm69_registers.h"
#include "esphome/components/spi/spi.h"

#ifdef ESPHOME_LOG_HAS_CONFIG
#include "esphome/core/log.h"
#else
// Mock logging для тестування
#define ESP_LOGCONFIG(tag, ...) printf(__VA_ARGS__); printf("\n")
#define ESP_LOGE(tag, ...) printf("ERROR: "); printf(__VA_ARGS__); printf("\n")
#define ESP_LOGW(tag, ...) printf("WARN: "); printf(__VA_ARGS__); printf("\n")
#define ESP_LOGI(tag, ...) printf("INFO: "); printf(__VA_ARGS__); printf("\n")
#define ESP_LOGD(tag, ...) printf("DEBUG: "); printf(__VA_ARGS__); printf("\n")
#define ESP_LOGV(tag, ...) printf("VERBOSE: "); printf(__VA_ARGS__); printf("\n")
#endif

namespace esphome {
namespace rfm69 {

static const char *const TAG = "rfm69";

// Modem configuration - FSK with 55.5kbps bit rate and 50kHz frequency deviation
static const uint8_t RFM69_MODEM_CONFIG[] = {
    RFM69_DATAMODUL_DATAMODE_PACKET | RFM69_DATAMODUL_MODULATIONTYPE_FSK | RFM69_DATAMODUL_MODULATIONSHAPING_00, // DataModul
    0x02, 0x40, // BitRate: 55.5 kbps
    0x03, 0x33, // Fdev: 50 kHz
    0x41,       // RxBw
    RFM69_PACKET1_FORMAT_VARIABLE | RFM69_PACKET1_DCFREE_WHITENING | RFM69_PACKET1_CRC_ON | RFM69_PACKET1_CRCAUTOCLEAR_ON | RFM69_PACKET1_ADRSFILTERING_NODEBROADCAST // PacketConfig1
};

void RFM69Component::setup() {
    ESP_LOGCONFIG(TAG, "Setting up RFM69...");

    // Initialize GPIO pins
    this->cs_pin_->setup();
    this->cs_pin_->digital_write(true);

    this->irq_pin_->setup();

    if (this->rst_pin_ != nullptr) {
        this->rst_pin_->setup();
        // Reset module
        this->rst_pin_->digital_write(true);
        delay(100);
        this->rst_pin_->digital_write(false);
        delay(100);
    }

    // Initialize SPI
    //SPI.begin();
	this->spi_setup();

    // Initialize radio
    if (!this->initialize_()) {
        ESP_LOGE(TAG, "Failed to initialize RFM69");
        this->mark_failed();
        return;
    }

    // Setup interrupt
    // Note: In real ESPHome, you would use attachInterrupt with proper pin and ISR
    // For now, we'll handle interrupts in the loop

    ESP_LOGCONFIG(TAG, "RFM69 setup complete");
}

void RFM69Component::loop() {
    // Handle any pending interrupts
    if (this->irq_flag_) {
        this->irq_flag_ = false;
        this->handle_interrupt_();
    }

    // Check for received data
    if (this->data_received_) {
        // Process received packet - in real implementation you might trigger callbacks here
        ESP_LOGD(TAG, "Packet received from node %d", this->current_packet_.header.sender);
    }
}

void RFM69Component::dump_config() {
    ESP_LOGCONFIG(TAG, "RFM69:");
    ESP_LOGCONFIG(TAG, "  Frequency: %.1f MHz", this->frequency_);
    ESP_LOGCONFIG(TAG, "  Network ID: %d", this->network_id_);
    ESP_LOGCONFIG(TAG, "  Node ID: %d", this->node_id_);
    ESP_LOGCONFIG(TAG, "  High Power: %s", this->high_power_ ? "YES" : "NO");
    ESP_LOGCONFIG(TAG, "  Power Level: %d dBm", this->power_level_dbm_);
    if (!this->aes_key_.empty()) {
        ESP_LOGCONFIG(TAG, "  Encryption: Enabled");
    }
    ESP_LOGCONFIG(TAG, "  Packets sent: %u", this->packets_sent_);
    ESP_LOGCONFIG(TAG, "  Packets received: %u", this->packets_received_);
    ESP_LOGCONFIG(TAG, "  Packets lost: %u", this->packets_lost_);
}

bool RFM69Component::initialize_() {
    ESP_LOGD(TAG, "Initializing RFM69 radio");

    // Set standby mode
    if (!this->set_radio_mode(RFM69RadioMode::STDBY)) {
        return false;
    }

    // Check communication with radio
    uint8_t version = this->read_register_(RFM69_REG_VERSION);
    if (version == 0x00 || version == 0xFF) {
        ESP_LOGE(TAG, "No RFM69 found! Version: 0x%02X", version);
        return false;
    }

    ESP_LOGI(TAG, "RFM69 version: 0x%02X", version);

    // Set configuration
    this->set_configuration_();

    // Set frequency
    this->set_frequency_(static_cast<uint32_t>(this->frequency_ * 1000000));

    // Set power level
    this->set_tx_power_level(this->power_level_dbm_);

    // Set network ID and node address
    this->write_register_(RFM69_REG_SYNCVALUE2, this->network_id_);
    this->write_register_(RFM69_REG_NODEADRS, this->node_id_);

    // Set encryption if key is provided
    if (!this->aes_key_.empty()) {
        this->set_encryption_(this->aes_key_.c_str());
    }

    // Perform sanity check
    if (!this->sanity_check_()) {
        ESP_LOGE(TAG, "RFM69 sanity check failed");
        return false;
    }

    // Set to RX mode
    return this->set_radio_mode(RFM69RadioMode::RX);
}

bool RFM69Component::sanity_check_() {
    bool result = true;
    result &= this->read_register_(RFM69_REG_RSSITHRESH) == RFM69_RSSITHRESH_VALUE;
    result &= this->read_register_(RFM69_REG_SYNCVALUE1) == RFM69_SYNCVALUE1;
    result &= this->read_register_(RFM69_REG_SYNCVALUE2) == this->network_id_;
    return result;
}

uint8_t RFM69Component::read_register_(uint8_t addr) {
    this->cs_pin_->digital_write(false);
    SPI.transfer(addr & RFM69_READ_REGISTER);
    uint8_t value = SPI.transfer(0);
    this->cs_pin_->digital_write(true);
    return value;
}

void RFM69Component::write_register_(uint8_t addr, uint8_t value) {
    this->cs_pin_->digital_write(false);
    SPI.transfer(addr | RFM69_WRITE_REGISTER);
    SPI.transfer(value);
    this->cs_pin_->digital_write(true);
}

uint8_t RFM69Component::burst_read_register_(uint8_t addr, uint8_t *buffer, uint8_t len) {
    this->cs_pin_->digital_write(false);
    uint8_t status = SPI.transfer(addr & RFM69_READ_REGISTER);
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0);
    }
    this->cs_pin_->digital_write(true);
    return status;
}

void RFM69Component::burst_write_register_(uint8_t addr, const uint8_t *buffer, uint8_t len) {
    this->cs_pin_->digital_write(false);
    SPI.transfer(addr | RFM69_WRITE_REGISTER);
    for (uint8_t i = 0; i < len; i++) {
        SPI.transfer(buffer[i]);
    }
    this->cs_pin_->digital_write(true);
}

bool RFM69Component::set_radio_mode(RFM69RadioMode new_mode) {
    if (this->radio_mode_ == new_mode) {
        return false; // No change needed
    }

    uint8_t reg_mode;

    switch (new_mode) {
        case RFM69RadioMode::STDBY:
            reg_mode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_STANDBY;
            ESP_LOGV(TAG, "Radio set to standby mode");
            break;

        case RFM69RadioMode::SLEEP:
            reg_mode = RFM69_OPMODE_SEQUENCER_OFF | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_SLEEP;
            ESP_LOGV(TAG, "Radio set to sleep mode");
            break;

        case RFM69RadioMode::RX:
            this->data_received_ = false;
            this->ack_received_ = false;
            reg_mode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_RECEIVER;
            this->write_register_(RFM69_REG_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0_01); // PayloadReady on DIO0
            this->set_high_power_regs_(false);
            ESP_LOGV(TAG, "Radio set to RX mode");
            break;

        case RFM69RadioMode::TX:
            reg_mode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_TRANSMITTER;
            this->write_register_(RFM69_REG_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0_00); // PacketSent on DIO0
            this->set_high_power_regs_(this->power_level_dbm_ >= RFM69_HIGH_POWER_DBM);
            ESP_LOGV(TAG, "Radio set to TX mode");
            break;

        case RFM69RadioMode::SYNTH:
            reg_mode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_SYNTHESIZER;
            ESP_LOGV(TAG, "Radio set to synth mode");
            break;

        default:
            reg_mode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_STANDBY;
            ESP_LOGV(TAG, "Radio set to standby mode (default)");
            break;
    }

    // Set new mode
    this->write_register_(RFM69_REG_OPMODE, reg_mode);

    // Wait for mode ready if waking from sleep
    if (this->radio_mode_ == RFM69RadioMode::SLEEP) {
        if (!this->is_mode_ready_()) {
            return false;
        }
    }

    this->radio_mode_ = new_mode;
    return true;
}

bool RFM69Component::is_mode_ready_() {
    uint32_t start = millis();
    while (!(this->read_register_(RFM69_REG_IRQFLAGS1) & RFM69_IRQFLAGS1_MODEREADY)) {
        if (millis() - start > RFM69_MODE_READY_TIMEOUT_MS) {
            return false;
        }
        delay(1);
    }
    return true;
}

void RFM69Component::set_frequency_(uint32_t frequency_hz) {
    const uint32_t freq_hz = static_cast<uint32_t>(frequency_hz / RFM69_FSTEP);
    this->write_register_(RFM69_REG_FRFMSB, static_cast<uint8_t>((freq_hz >> 16) & 0xFF));
    this->write_register_(RFM69_REG_FRFMID, static_cast<uint8_t>((freq_hz >> 8) & 0xFF));
    this->write_register_(RFM69_REG_FRFLSB, static_cast<uint8_t>(freq_hz & 0xFF));
}

bool RFM69Component::set_tx_power_level(RFM69PowerLevel new_power_level) {
    // Clamp power level to valid range
    new_power_level = std::max(static_cast<RFM69PowerLevel>(RFM69_MIN_POWER_LEVEL_DBM), new_power_level);
    new_power_level = std::min(static_cast<RFM69PowerLevel>(RFM69_MAX_POWER_LEVEL_DBM), new_power_level);

    if (this->power_level_dbm_ == new_power_level) {
        return false; // No change needed
    }

    this->power_level_dbm_ = new_power_level;
    uint8_t pa_level;

#ifndef MY_RFM69HW
    // RFM69W: -18dBm to +13dBm, PA0, offset 18
    pa_level = RFM69_PALEVEL_PA0_ON | static_cast<uint8_t>(new_power_level + 18);
#else
    // RFM69HW
    if (new_power_level <= 13) {
        // -2dBm to +13dBm, PA1, offset 18
        pa_level = RFM69_PALEVEL_PA1_ON | static_cast<uint8_t>(new_power_level + 18);
    } else if (new_power_level >= RFM69_HIGH_POWER_DBM) {
        // +18dBm to +20dBm, PA1 and PA2, offset 11
        pa_level = RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | static_cast<uint8_t>(new_power_level + 11);
    } else {
        // +14dBm to +17dBm, PA1 and PA2, offset 14
        pa_level = RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | static_cast<uint8_t>(new_power_level + 14);
    }
#endif

    this->write_register_(RFM69_REG_PALEVEL, pa_level);
    ESP_LOGD(TAG, "TX power level set to %d dBm", new_power_level);
    return true;
}

uint8_t RFM69Component::get_tx_power_percent() const {
    return static_cast<uint8_t>(100.0f * (this->power_level_dbm_ - RFM69_MIN_POWER_LEVEL_DBM) /
                               (RFM69_MAX_POWER_LEVEL_DBM - RFM69_MIN_POWER_LEVEL_DBM));
}

bool RFM69Component::set_tx_power_percent(uint8_t power_percent) {
    power_percent = std::min(power_percent, static_cast<uint8_t>(100));
    const RFM69PowerLevel new_power_level = static_cast<RFM69PowerLevel>(
        RFM69_MIN_POWER_LEVEL_DBM + (RFM69_MAX_POWER_LEVEL_DBM - RFM69_MIN_POWER_LEVEL_DBM) * (power_percent / 100.0f));
    return this->set_tx_power_level(new_power_level);
}

void RFM69Component::set_high_power_regs_(bool on_off) {
#ifdef MY_RFM69HW
    this->write_register_(RFM69_REG_OCP, on_off ? RFM69_OCP_OFF : RFM69_OCP_ON | RFM69_OCP_TRIM_95);
    this->write_register_(RFM69_REG_TESTPA1, on_off ? 0x5D : 0x55);
    this->write_register_(RFM69_REG_TESTPA2, on_off ? 0x7C : 0x70);
#else
    (void)on_off; // Suppress unused parameter warning
#endif
}

bool RFM69Component::send_message(uint8_t to_address, const std::string &data, bool no_ack) {
    return this->send_message(to_address, reinterpret_cast<const uint8_t*>(data.c_str()),
                             static_cast<uint8_t>(data.length()), no_ack);
}

bool RFM69Component::send_message(uint8_t to_address, const uint8_t *data, uint8_t len, bool no_ack) {
    if (this->send_with_retry_(to_address, data, len, no_ack)) {
        this->packets_sent_++;
        ESP_LOGD(TAG, "Message sent to %d, length: %d", to_address, len);
        return true;
    } else {
        this->packets_lost_++;
        ESP_LOGW(TAG, "Failed to send message to %d", to_address);
        return false;
    }
}

bool RFM69Component::send_with_retry_(uint8_t recipient, const uint8_t *buffer, uint8_t buffer_size, bool no_ack) {
    for (uint8_t retry = 0; retry < RFM69_RETRIES; retry++) {
        ESP_LOGV(TAG, "Sending to %d, seq=%d, retry=%d", recipient, this->tx_sequence_number_, retry);

        RFM69ControlFlags flags = 0;
        this->set_ack_requested_(flags, !no_ack);
        this->set_ack_rssi_report_(flags, this->atc_enabled_);

        if (!this->send_packet_(recipient, buffer, buffer_size, flags, !retry)) {
            ESP_LOGW(TAG, "Send failed, no IRQ");
        }

        if (no_ack) {
            return true; // No ACK requested, assume success
        }

        // Wait for ACK
        uint32_t start_time = millis();
        while (millis() - start_time < RFM69_RETRY_TIMEOUT_MS && !this->data_received_) {
            this->loop(); // Process any incoming packets

            if (this->ack_received_) {
                // Check if ACK is for us
                if (this->current_packet_.header.sender == recipient &&
                    this->current_packet_.ack.sequence_number == this->tx_sequence_number_) {
                    ESP_LOGV(TAG, "ACK received from %d, seq=%d", recipient, this->tx_sequence_number_);
                    this->ack_received_ = false;
                    this->set_radio_mode(RFM69RadioMode::RX);
                    return true;
                }
                this->ack_received_ = false;
            }
        }

        ESP_LOGV(TAG, "No ACK received, retry %d", retry);
    }

    return false;
}

bool RFM69Component::send_packet_(uint8_t recipient, const uint8_t *data, uint8_t len,
                                 RFM69ControlFlags flags, bool increase_sequence_counter) {
    // Prepare packet
    RFM69Packet packet;
    packet.header.version = RFM69_PACKET_HEADER_VERSION;
    packet.header.sender = this->node_id_;
    packet.header.recipient = recipient;
    packet.payload_len = std::min(len, static_cast<uint8_t>(RFM69_MAX_PACKET_LEN - sizeof(RFM69Header)));
    packet.header.control_flags = flags;
    memcpy(packet.payload, data, packet.payload_len);
    packet.header.packet_len = packet.payload_len + sizeof(RFM69Header) - 1; // -1 for length byte

    return this->send_frame_(&packet, increase_sequence_counter);
}

bool RFM69Component::send_frame_(RFM69Packet *packet, bool increase_sequence_counter) {
    // Check if channel is free (CSMA)
    this->set_radio_mode(RFM69RadioMode::RX);
    delay(1); // Allow RSSI measurement

    uint32_t csma_start = millis();
    while (!this->channel_free_() && (millis() - csma_start) < 500) { // 500ms timeout
        delay(1);
    }

    // Set to standby to load FIFO
    this->set_radio_mode(RFM69RadioMode::STDBY);

    if (increase_sequence_counter) {
        this->tx_sequence_number_++;
    }

    // Clear FIFO and flags
    this->clear_fifo_();

    // Assign sequence number
    packet->header.sequence_number = this->tx_sequence_number_;

    // Write packet to FIFO
    const uint8_t final_len = packet->payload_len + sizeof(RFM69Header);
    this->burst_write_register_(RFM69_REG_FIFO, packet->data, final_len);

    // Send packet
    this->set_radio_mode(RFM69RadioMode::TX);

    // Wait for transmission complete
    uint32_t tx_start = millis();
    this->irq_flag_ = false;
    while (!this->irq_flag_ && (millis() - tx_start < 2000)) { // 2s timeout
        // In real implementation, this would be handled by interrupt
        // For testing, we simulate the IRQ flag
        delay(10);
        uint8_t irq_flags = this->read_register_(RFM69_REG_IRQFLAGS2);
        if (irq_flags & RFM69_IRQFLAGS2_PACKETSENT) {
            this->irq_flag_ = true;
        }
    }

    return this->irq_flag_;
}

void RFM69Component::clear_fifo_() {
    this->write_register_(RFM69_REG_IRQFLAGS2, RFM69_IRQFLAGS2_FIFOOVERRUN);
}

bool RFM69Component::channel_free_() {
    RFM69RSSI rssi = this->read_rssi_(false);
    return this->rssi_internal_to_dbm_(rssi) < -90; // Channel free if RSSI < -90 dBm
}

RFM69RSSI RFM69Component::read_rssi_(bool force_trigger) {
    (void)force_trigger; // RSSI is always available in continuous mode
    return this->read_register_(RFM69_REG_RSSIVALUE);
}

int16_t RFM69Component::rssi_internal_to_dbm_(RFM69RSSI internal_rssi) const {
    return static_cast<int16_t>(-(internal_rssi / 2));
}

RFM69RSSI RFM69Component::rssi_dbm_to_internal_(int16_t rssi_dbm) const {
    return static_cast<RFM69RSSI>(-rssi_dbm * 2);
}

int16_t RFM69Component::get_receiving_rssi() const {
    return this->rssi_internal_to_dbm_(this->current_packet_.rssi);
}

int16_t RFM69Component::get_sending_rssi() const {
    if (this->get_ack_rssi_report_(this->current_packet_.header.control_flags)) {
        return this->rssi_internal_to_dbm_(this->current_packet_.ack.rssi);
    }
    return 127; // Invalid value
}

bool RFM69Component::is_message_available() {
    if (this->data_received_) {
        return true;
    } else if (this->radio_mode_ == RFM69RadioMode::TX) {
        return false;
    } else if (this->radio_mode_ != RFM69RadioMode::RX) {
        this->set_radio_mode(RFM69RadioMode::RX);
    }
    return false;
}

uint8_t RFM69Component::receive_message(uint8_t *buffer, uint8_t max_len) {
    if (!this->data_received_) {
        return 0;
    }

    const uint8_t payload_len = std::min(this->current_packet_.payload_len, max_len);
    const uint8_t sender = this->current_packet_.header.sender;
    const RFM69SequenceNumber sequence_number = this->current_packet_.header.sequence_number;
    const RFM69ControlFlags control_flags = this->current_packet_.header.control_flags;
    const RFM69RSSI rssi = this->current_packet_.rssi;

    if (buffer != nullptr) {
        memcpy(buffer, this->current_packet_.payload, payload_len);
    }

    // Clear data flag
    this->data_received_ = false;
    this->packets_received_++;

    // Send ACK if requested
    if (this->get_ack_requested_(control_flags) && !this->get_ack_received_(control_flags)) {
        this->send_ack_(sender, sequence_number, rssi);
    }

    return payload_len;
}

void RFM69Component::send_ack_(uint8_t recipient, RFM69SequenceNumber sequence_number, RFM69RSSI rssi) {
    ESP_LOGV(TAG, "Sending ACK to %d, seq=%d, RSSI=%d", recipient, sequence_number,
             this->rssi_internal_to_dbm_(rssi));

    RFM69Ack ack;
    ack.sequence_number = sequence_number;
    ack.rssi = rssi;

    RFM69ControlFlags flags = 0;
    this->set_ack_received_(flags, true);
    this->set_ack_rssi_report_(flags, true);

    this->send_packet_(recipient, reinterpret_cast<uint8_t*>(&ack), sizeof(RFM69Ack), flags);
}

void RFM69Component::handle_interrupt_() {
    uint8_t irq_flags2 = this->read_register_(RFM69_REG_IRQFLAGS2);

    if (this->radio_mode_ == RFM69RadioMode::RX && (irq_flags2 & RFM69_IRQFLAGS2_PAYLOADREADY)) {
        this->set_radio_mode(RFM69RadioMode::STDBY);

        if (irq_flags2 & RFM69_IRQFLAGS2_FIFOLEVEL) {
            // Read packet
            uint8_t *current = reinterpret_cast<uint8_t*>(&this->current_packet_);
            bool header_read = false;
            uint8_t reading_length = sizeof(RFM69Header);

            this->cs_pin_->digital_write(false);
            SPI.transfer(RFM69_REG_FIFO & RFM69_READ_REGISTER);

            while (reading_length--) {
                *current++ = SPI.transfer(0);

                if (!reading_length && !header_read) {
                    header_read = true;
                    if (this->current_packet_.header.version >= RFM69_MIN_PACKET_HEADER_VERSION) {
                        reading_length = std::min(
                            static_cast<uint8_t>(this->current_packet_.header.packet_len - (sizeof(RFM69Header) - 1)),
                            static_cast<uint8_t>(RFM69_MAX_PACKET_LEN));
                        this->current_packet_.payload_len = reading_length;
                        this->ack_received_ = this->get_ack_received_(this->current_packet_.header.control_flags);
                        this->data_received_ = !this->ack_received_;
                    }
                }
            }

            this->cs_pin_->digital_write(true);
        }

        this->current_packet_.rssi = this->read_rssi_();
        // Radio remains in standby until packet is processed
    } else {
        // Back to RX mode
        this->set_radio_mode(RFM69RadioMode::RX);
    }
}

bool RFM69Component::sleep() {
    return this->set_radio_mode(RFM69RadioMode::SLEEP);
}

bool RFM69Component::standby() {
    return this->set_radio_mode(RFM69RadioMode::STDBY);
}

void RFM69Component::set_encryption_(const char *key) {
    this->set_radio_mode(RFM69RadioMode::STDBY);

    if (key != nullptr) {
        this->burst_write_register_(RFM69_REG_AESKEY1, reinterpret_cast<const uint8_t*>(key), 16);
    }

    this->write_register_(RFM69_REG_PACKETCONFIG2,
                         (this->read_register_(RFM69_REG_PACKETCONFIG2) & 0xFE) |
                         (key ? RFM69_PACKET2_AES_ON : RFM69_PACKET2_AES_OFF));
}

void RFM69Component::set_configuration_() {
    // Configuration table
    const uint8_t config[][2] = {
        {RFM69_REG_OPMODE, RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_STANDBY},
        {RFM69_REG_DATAMODUL, RFM69_MODEM_CONFIG[0]},
        {RFM69_REG_BITRATEMSB, RFM69_MODEM_CONFIG[1]},
        {RFM69_REG_BITRATELSB, RFM69_MODEM_CONFIG[2]},
        {RFM69_REG_FDEVMSB, RFM69_MODEM_CONFIG[3]},
        {RFM69_REG_FDEVLSB, RFM69_MODEM_CONFIG[4]},
        {RFM69_REG_LNA, RFM69_LNA_ZIN_200 | RFM69_LNA_CURRENTGAIN},
        {RFM69_REG_RXBW, RFM69_MODEM_CONFIG[5]},
        {RFM69_REG_AFCBW, RFM69_MODEM_CONFIG[5]},
        {RFM69_REG_DIOMAPPING2, RFM69_DIOMAPPING2_CLKOUT_OFF},
        {RFM69_REG_IRQFLAGS2, RFM69_IRQFLAGS2_FIFOOVERRUN},
        {RFM69_REG_RSSITHRESH, RFM69_RSSITHRESH_VALUE},
        {RFM69_REG_PREAMBLEMSB, RFM69_PREAMBLESIZE_MSB_VALUE},
        {RFM69_REG_PREAMBLELSB, RFM69_PREAMBLESIZE_LSB_VALUE},
        {RFM69_REG_SYNCCONFIG, RFM69_SYNC_ON | RFM69_SYNC_FIFOFILL_AUTO | RFM69_SYNC_SIZE_2 | RFM69_SYNC_TOL_0},
        {RFM69_REG_SYNCVALUE1, RFM69_SYNCVALUE1},
        {RFM69_REG_PACKETCONFIG1, RFM69_MODEM_CONFIG[6]},
        {RFM69_REG_PAYLOADLENGTH, RFM69_MAX_PACKET_LEN},
        {RFM69_REG_BROADCASTADRS, RFM69_BROADCAST_ADDRESS},
        {RFM69_REG_FIFOTHRESH, RFM69_FIFOTHRESH_TXSTART_FIFONOTEMPTY | (sizeof(RFM69Header) - 1)},
        {RFM69_REG_PACKETCONFIG2, RFM69_PACKET2_RXRESTARTDELAY_2BITS | RFM69_PACKET2_AUTORXRESTART_OFF | RFM69_PACKET2_AES_OFF},
        {RFM69_REG_TESTDAGC, RFM69_DAGC_IMPROVED_LOWBETA0},
        {255, 0} // End marker
    };

    for (uint8_t i = 0; config[i][0] != 255; i++) {
        this->write_register_(config[i][0], config[i][1]);
    }
}

}  // namespace rfm69
}  // namespace esphome