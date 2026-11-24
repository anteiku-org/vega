#include "LoRaComm.h"

// SX1276 Registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05

LoRaComm::LoRaComm()
    : spiLoRa(VSPI), lastPacketTime(0), failsafeActive(true),
      targetThrottle(1000.0f), targetRoll(0.0f), targetPitch(0.0f), targetYawRate(0.0f),
      armedRequested(false), angleMode(true) {
}

void LoRaComm::writeRegister(uint8_t reg, uint8_t val) {
    digitalWrite(PIN_LORA_CS, LOW);
    spiLoRa.transfer(reg | 0x80); // Bit 7 = 1 for write
    spiLoRa.transfer(val);
    digitalWrite(PIN_LORA_CS, HIGH);
}

uint8_t LoRaComm::readRegister(uint8_t reg) {
    digitalWrite(PIN_LORA_CS, LOW);
    spiLoRa.transfer(reg & 0x7F); // Bit 7 = 0 for read
    uint8_t val = spiLoRa.transfer(0x00);
    digitalWrite(PIN_LORA_CS, HIGH);
    return val;
}

bool LoRaComm::begin() {
    pinMode(PIN_LORA_CS, OUTPUT);
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_CS, HIGH);

    // Hardware reset pulse
    digitalWrite(PIN_LORA_RST, LOW);
    delay(10);
    digitalWrite(PIN_LORA_RST, HIGH);
    delay(10);

    spiLoRa.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12) {
        // Module not found or communication failed
        return false;
    }

    // Put into sleep mode to configure LoRa mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    delay(5);

    // Set carrier frequency to 433 MHz (433E6 / 61.035 Hz = 0x6C8000)
    uint64_t frf = ((uint64_t)LORA_FREQUENCY << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));

    // Base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // Modem config: BW 250kHz, CR 4/5, SF7 (High speed for low RC latency)
    writeRegister(REG_MODEM_CONFIG_1, 0x82); // 250 kHz, CR 4/5
    writeRegister(REG_MODEM_CONFIG_2, 0x74); // SF7, CRC enabled
    writeRegister(REG_MODEM_CONFIG_3, 0x04); // LNA AGC on

    // PA Config: High power (17dBm on PA_BOOST)
    writeRegister(REG_PA_CONFIG, 0x8F);

    // Enter continuous receive mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    lastPacketTime = millis();
    return true;
}

void LoRaComm::update() {
    // 1. Check for incoming Radio Command Packet
    uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
    if (irqFlags & 0x40) { // RxDone
        writeRegister(REG_IRQ_FLAGS, 0xFF); // Clear IRQ flags

        uint8_t currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        uint8_t bytesReceived = readRegister(REG_RX_NB_BYTES);

        if (bytesReceived == sizeof(RadioCommandPacket)) {
            writeRegister(REG_FIFO_ADDR_PTR, currentAddr);
            RadioCommandPacket pkt;
            uint8_t* p = (uint8_t*)&pkt;

            digitalWrite(PIN_LORA_CS, LOW);
            spiLoRa.transfer(REG_FIFO & 0x7F);
            for (size_t i = 0; i < sizeof(RadioCommandPacket); i++) {
                p[i] = spiLoRa.transfer(0x00);
            }
            digitalWrite(PIN_LORA_CS, HIGH);

            // Compute XOR checksum
            uint8_t cs = 0;
            for (size_t i = 0; i < sizeof(RadioCommandPacket) - 1; i++) {
                cs ^= p[i];
            }

            if (cs == pkt.checksum) {
                targetThrottle = clampFloat((float)pkt.throttle, 1000.0f, 2000.0f);
                targetRoll = (float)pkt.roll / 10.0f;
                targetPitch = (float)pkt.pitch / 10.0f;
                targetYawRate = (float)pkt.yaw / 10.0f;
                armedRequested = (pkt.flags & 0x01);
                angleMode = (pkt.flags & 0x02) != 0;

                lastPacketTime = millis();
                failsafeActive = false;
            }
        }
    }

    // 2. Failsafe Watchdog Check
    if (millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
        failsafeActive = true;
        targetThrottle = 1000.0f;
        targetRoll = 0.0f;
        targetPitch = 0.0f;
        targetYawRate = 0.0f;
        armedRequested = false;
    }
}

void LoRaComm::sendTelemetry(float roll, float pitch, float yaw, float altitude, float vbat, int sats, bool armed, bool failsafe) {
    DroneTelemetryPacket pkt;
    pkt.roll = (int16_t)(roll * 10.0f);
    pkt.pitch = (int16_t)(pitch * 10.0f);
    pkt.yaw = (int16_t)(yaw * 10.0f);
    pkt.altitudeDm = (int16_t)(altitude * 10.0f);
    pkt.batteryMv = (uint16_t)(vbat * 1000.0f);
    pkt.gpsSats = (uint8_t)clampFloat((float)sats, 0.0f, 255.0f);
    pkt.statusFlags = (armed ? 0x01 : 0x00) | (failsafe ? 0x02 : 0x00);

    uint8_t* p = (uint8_t*)&pkt;
    uint8_t cs = 0;
    for (size_t i = 0; i < sizeof(DroneTelemetryPacket) - 1; i++) {
        cs ^= p[i];
    }
    pkt.checksum = cs;

    // Send packet
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);

    digitalWrite(PIN_LORA_CS, LOW);
    spiLoRa.transfer(REG_FIFO | 0x80);
    for (size_t i = 0; i < sizeof(DroneTelemetryPacket); i++) {
        spiLoRa.transfer(p[i]);
    }
    digitalWrite(PIN_LORA_CS, HIGH);

    writeRegister(REG_PAYLOAD_LENGTH, sizeof(DroneTelemetryPacket));
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    // Return to RX after transmitting
    unsigned long start = millis();
    while ((readRegister(REG_IRQ_FLAGS) & 0x08) == 0) { // Wait for TxDone
        if (millis() - start > 50) break;
    }
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
